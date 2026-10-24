use std::{error::Error, net::IpAddr, path::PathBuf};
use tower_http::services::ServeDir;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let mut bind: IpAddr = "127.0.0.1".parse()?;
    let mut port: u16 = 3000;
    let mut assets = PathBuf::from("web/dist");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--bind" => bind = args.next().ok_or("missing bind address")?.parse()?,
            "--port" => port = args.next().ok_or("missing port")?.parse()?,
            "--assets" => assets = args.next().ok_or("missing asset directory")?.into(),
            "--help" | "-h" => {
                println!("gyro-server [--bind 127.0.0.1] [--port 3000] [--assets web/dist]");
                return Ok(());
            }
            _ => return Err(format!("unknown option {arg}").into()),
        }
    }
    if !assets.join("index.html").is_file() {
        return Err(format!(
            "built assets missing at {}; build web/ first or pass --assets",
            assets.display()
        )
        .into());
    }
    let assets = assets.canonicalize()?;
    let listener = tokio::net::TcpListener::bind((bind, port)).await?;
    println!(
        "Gyroscope lab: http://{} (assets: {})",
        listener.local_addr()?,
        assets.display()
    );
    let app = axum::Router::new().fallback_service(ServeDir::new(assets));
    axum::serve(listener, app)
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}
