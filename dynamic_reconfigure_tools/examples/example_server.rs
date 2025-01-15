/// example dynamic reconfigure server
use roslibrust::ros1::NodeHandle;
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

    dr.add_bool_param("enable", false, "fake enable");
    dr.add_str_param("foo", "bar", "some string param");
    dr.add_double_param("floating", 3.0, 0.0, 16.0, "a float");
    dr.add_str_param("another", "bar2", "another string param");
    dr.add_int_param("int_val", 50, -20, 200, "int param");

    let mut dr_enums = Vec::new();
    dr_enums.push(("foo", "somme foo"));
    dr_enums.push(("bar", "somme bar"));
    dr_enums.push(("tmp", "somme tmp enum"));
    dr.add_enum_param("my enum", 1, dr_enums, "select amongst these");

    tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    dr.init().await?;

    let wait_millis = 200;
    let mut update_interval =
        tokio::time::interval(tokio::time::Duration::from_millis(wait_millis));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                tracing::info!("ctrl-c, exiting");
                break;
            }
            _ = update_interval.tick() => {
            //_ = dr.update_receiver.recv() => {
                // let stamp = tf_util::stamp_now();
                match dr.update().await {
                    Ok(true) => {
                        let val = dr.get_str("foo");
                        tracing::info!("foo val: '{val:?}'");
                        // this will fail
                        let val = dr.get_str("none_named_this");
                        tracing::info!("no value expected: '{val:?}'");
                        let val = dr.get_double("floating");
                        tracing::info!("floating val: '{val:?}'");
                        let val = dr.get_int("int_val");
                        tracing::info!("int val: '{val:?}'");
                    }
                    Ok(false) => {}
                    Err(err) => {
                        tracing::warn!("{err:?}");
                    }
                }
            }
        }
    }

    Ok(())
}
