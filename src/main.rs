use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context;
use jetarm_mcp::{actions::ActionStore, arm::Arm, board::Board, config::Config, server::JetArmServer};
use rmcp::transport::streamable_http_server::{
    StreamableHttpServerConfig, StreamableHttpService, session::local::LocalSessionManager,
};
use tokio_util::sync::CancellationToken;

const USAGE: &str = "用法：jetarm-mcp [--config <config.toml>]\n\
    不给 --config 时依次尝试 ~/.config/jetarm-mcp/config.toml，再用内置缺省值。";

fn load_config() -> anyhow::Result<(Config, String)> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [] => {
            let p = dirs::config_dir().map(|d| d.join("jetarm-mcp/config.toml"));
            match p {
                Some(p) if p.exists() => Ok((Config::load(&p)?, p.display().to_string())),
                _ => Ok((Config::default(), "内置缺省值".into())),
            }
        }
        [flag, path] if flag == "--config" => {
            let p = PathBuf::from(path);
            Ok((Config::load(&p)?, p.display().to_string()))
        }
        _ => {
            println!("{USAGE}");
            std::process::exit(if args.iter().any(|a| a == "-h" || a == "--help") { 0 } else { 2 });
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()))
        .init();

    let (cfg, source) = load_config()?;
    tracing::info!("配置来源：{source}");
    let cfg = Arc::new(cfg);

    let s = &cfg.serial;
    let board = Board::open_serial(
        s.device.clone(),
        s.baud,
        Duration::from_secs_f64(s.read_timeout),
        s.read_retries,
    );
    if !board.wait_connected(Duration::from_secs(2)) {
        tracing::warn!("串口 {} 暂未连上，服务照常启动，后台每 2 秒重连", s.device);
    }
    let arm = Arc::new(Arm::new(Arc::new(board), cfg.clone()));
    let actions = Arc::new(ActionStore::new(cfg.actions_dir()));
    tracing::info!("动作组目录：{}", cfg.actions_dir().display());

    // 启动时只读状态，不做任何运动
    let st = arm.state(false).await;
    tracing::info!("当前末端位姿：{:?}", st.end_effector);

    let ct = CancellationToken::new();
    let service: StreamableHttpService<JetArmServer, LocalSessionManager> = StreamableHttpService::new(
        {
            let (arm, actions) = (arm.clone(), actions.clone());
            move || Ok(JetArmServer::new(arm.clone(), actions.clone()))
        },
        Default::default(),
        StreamableHttpServerConfig::default()
            .disable_allowed_hosts()
            .with_cancellation_token(ct.child_token()),
    );
    let router = axum::Router::new().nest_service("/mcp", service);
    let addr = format!("{}:{}", cfg.server.bind, cfg.server.port);
    let listener = tokio::net::TcpListener::bind(&addr).await.with_context(|| format!("监听 {addr}"))?;
    tracing::warn!("服务对局域网开放、无认证：任何能访问 {addr} 的客户端都能控制机械臂");
    tracing::info!("MCP 端点：http://{addr}/mcp");

    axum::serve(listener, router)
        .with_graceful_shutdown(async move {
            let _ = tokio::signal::ctrl_c().await;
            tracing::info!("收到退出信号");
            ct.cancel();
        })
        .await?;
    Ok(())
}
