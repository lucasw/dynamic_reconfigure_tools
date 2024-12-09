/// example dynamic reconfigure server
use roslibrust::ros1::NodeHandle;
use roslibrust_util::dynamic_reconfigure;
use std::collections::HashMap;
use tracing_subscriber;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    let tracing_sub = tracing_subscriber::fmt().finish();
    tracing::subscriber::set_global_default(tracing_sub)?;

    let mut params = HashMap::<String, String>::new();
    params.insert("_name".to_string(), "dynrec_server".to_string());
    let mut remaps = HashMap::<String, String>::new();

    let (_ns, full_node_name, _remaining_args) =
        roslibrust_util::get_params_remaps(&mut params, &mut remaps);

    let ros_master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
    let nh = NodeHandle::new(&ros_master_uri, &full_node_name).await?;
    tracing::info!("connected to roscore at {ros_master_uri}");

    // Dynamic reconfigure service and topics
    let mut dr = dynamic_reconfigure_tools::DynamicReconfigure::new(&nh, &full_node_name).await?;

    dr.add_str_param("foo", "bar", "some string param");
    dr.add_str_param("another", "bar2", "another string param");

    tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    dr.init();

    let wait_millis = 100;
    let mut update_interval =
        tokio::time::interval(tokio::time::Duration::from_millis(wait_millis));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                tracing::info!("ctrl-c, exiting");
                break;
            }
            _ = update_interval.tick() => {
                // let stamp = tf_util::stamp_now();
            }
        }
    }

    Ok(())
}
