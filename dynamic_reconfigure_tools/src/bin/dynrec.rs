/// call a dynamic reconfigure server from the command line
///
/// dynrec /example_server_node2 double_param "5.0"
use dynamic_reconfigure_tools::ParamType;
use roslibrust_util::dynamic_reconfigure;
use std::collections::HashMap;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    let tracing_sub = tracing_subscriber::fmt().finish();
    tracing::subscriber::set_global_default(tracing_sub)?;

    let mut params = HashMap::<String, String>::new();
    params.insert("_name".to_string(), "dynrec".to_string());
    let mut remaps = HashMap::<String, String>::new();

    let (_ns, full_node_name, remaining_args) =
        roslibrust_util::get_params_remaps(&mut params, &mut remaps);

    let ros_master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
    let nh = NodeHandle::new(&ros_master_uri, &full_node_name).await?;
    tracing::info!("connected to roscore at {ros_master_uri}");

    // TODO(lucasw) prefix ns to server name if no leading slash
    let partial_server_name = &remaining_args[1];
    let full_server_name = dynamic_reconfigure_tools::get_server_name(partial_server_name);

    let client = nh
        .service_client::<dynamic_reconfigure::Reconfigure>(&full_server_name)
        .await
        .expect("can't connect to {full_server_name}");
    tracing::info!("connected to dynamic reconfigure server {full_server_name}");

    let name = remaining_args[2].to_string();

    let raw_value = &remaining_args[3];

    let (request, param_type) = dynamic_reconfigure_tools::raw_value_to_request(raw_value, &name);

    let response = client.call(&request).await?;
    // tracing::info!("Got response: {:?}", response);

    // show if request changed the value
    let mut name_in_response = false;
    match param_type {
        // TODO(lucasw) could construct a hashmap of param type and config values
        ParamType::INT => {
            for intv in &response.config.ints {
                if intv.name == name {
                    tracing::info!("{param_type:?} {name} set to {}", intv.value);
                    name_in_response = true;
                    break;
                }
            }
        }
        ParamType::DOUBLE => {
            for doublev in &response.config.doubles {
                if doublev.name == name {
                    tracing::info!("{param_type:?} {name} set to {}", doublev.value);
                    name_in_response = true;
                    break;
                }
            }
        }
        ParamType::BOOL => {
            for boolv in &response.config.bools {
                if boolv.name == name {
                    tracing::info!("{param_type:?} {name} set to {}", boolv.value);
                    name_in_response = true;
                    break;
                }
            }
        }
        ParamType::STR => {
            for strv in &response.config.strs {
                if strv.name == name {
                    tracing::info!("{param_type:?} {name} set to {}", strv.value);
                    name_in_response = true;
                    break;
                }
            }
        }
    }
    if !name_in_response {
        tracing::error!("{param_type:?} {name} not found {:?}", response.config);
    }

    Ok(())
}
