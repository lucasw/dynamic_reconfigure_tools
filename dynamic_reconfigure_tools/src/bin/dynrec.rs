/// call a dynamic reconfigure server from the command line

use tracing_subscriber;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    let tracing_sub = tracing_subscriber::fmt().finish();
    tracing::subscriber::set_global_default(tracing_sub)?;

    let nh = NodeHandle::new("http://localhost:11311", "service_client_rs").await?;
    tracing::info!("connected to roscore");

    let client = nh
        .service_client::<dynamic_reconfigure::Reconfigure>("/example_server_node2/set_parameters")
        .await?;

    let req = {
        let mut req = dynamic_reconfigure::ReconfigureRequest::default();
        let mut double = dynamic_reconfigure::DoubleParameter::default();
        double.name = "small_double".to_string();
        double.value = 0.123;
        req.config.doubles.push(double);
        req
    };

    let response = client.call(&req).await?;
    tracing::info!("Got response: {:?}", response);

    Ok(())
}
