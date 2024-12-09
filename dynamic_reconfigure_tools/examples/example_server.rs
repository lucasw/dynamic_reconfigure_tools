/// example dynamic reconfigure server
use roslibrust_util::dynamic_reconfigure;
use std::collections::HashMap;
use tracing_subscriber;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::{NodeHandle, Publisher};
    use std::sync::{Arc, Mutex};

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
    let full_server_name = dynamic_reconfigure_tools::get_server_name(&full_node_name);

    let mut config_description = dynamic_reconfigure::ConfigDescription::default();
    let config_state = Arc::new(Mutex::new(dynamic_reconfigure::Config::default()));
    // TODO(lucasw) set up a few parameters so they can be set
    {
        let mut config = config_state.lock().unwrap();
        let str_param = dynamic_reconfigure::StrParameter {
            name: "foo".to_string(),
            value: "bar".to_string(),
        };

        let mut group = dynamic_reconfigure::Group::default();
        group.name = "Default".to_string();
        let str_param_description = dynamic_reconfigure::ParamDescription {
            name: str_param.name.clone(),
            r#type: "str".to_string(),
            level: 1,
            description: "test str".to_string(),
            edit_method: "".to_string(),
        };
        group.parameters.push(str_param_description);

        // Don't need a max string but it's there anyhow, assume the min/max values don't matter
        config_description.min.strs.push(str_param.clone());
        config_description.max.strs.push(str_param.clone());
        config_description.dflt.strs.push(str_param.clone());
        config.strs.push(str_param);

        let mut group_state = dynamic_reconfigure::GroupState::default();
        group_state.name = group.name.clone();
        group_state.state = true;
        config_description.min.groups.push(group_state.clone());
        config_description.max.groups.push(group_state.clone());
        config_description.dflt.groups.push(group_state.clone());
        config_description.groups.push(group);

        config.groups.push(group_state);
    }
    tracing::info!("Initial config: {config_state:?}");

    // TODO(lucasw) need these to be latched, but maybe just regularly publishing will work
    let latch = true;
    let update_pub: Publisher<dynamic_reconfigure::Config> = nh
        .advertise(
            &format!("{}/parameter_updates", full_node_name.as_str()),
            3,
            latch,
        )
        .await?;
    let do_update_pub = Arc::new(Mutex::new(true));
    let description_pub: Publisher<dynamic_reconfigure::ConfigDescription> = nh
        .advertise(
            &format!("{}/parameter_descriptions", full_node_name.as_str()),
            3,
            latch,
        )
        .await?;

    let config_state_copy = config_state.clone();
    let do_update_pub_copy = do_update_pub.clone();
    let server_fn = move |request: dynamic_reconfigure::ReconfigureRequest| {
        tracing::info!("{request:?}");

        let mut config = config_state_copy.lock().unwrap();

        // TODO(lucasw) need a hashmap as base structure so don't have to do double for loops
        // - then convert it to and from Config as needed
        // for intv in request.config.ints {
        // }
        'outer: for req_strv in request.config.strs {
            for strv in &mut config.strs {
                if req_strv.name == strv.name {
                    if strv.value != req_strv.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            strv.name,
                            strv.value,
                            req_strv.value
                        );
                        strv.value = req_strv.value;
                        break 'outer;
                    }
                }
            }
        }
        *do_update_pub_copy.lock().unwrap() = true;

        Ok(dynamic_reconfigure::ReconfigureResponse {
            config: config.clone(),
        })
    };

    let _server_handle = nh
        .advertise_service::<dynamic_reconfigure::Reconfigure, _>(&full_server_name, server_fn)
        .await
        .expect("can't connect to {full_server_name}");
    tracing::info!("serving dynamic reconfigure server on {full_server_name}");

    // Setup a task to kill this process when ctrl_c comes in:
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0);
    });

    tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    let _ = description_pub.publish(&config_description);

    loop {
        // let config_state = config_state.lock().unwrap().clone();
        // tracing::info!("Current value of config: {config_state:?}");
        {
            // TODO(lucasw) can't await in the server_fn above, so doing this arc mutex - is there
            // a better way?
            let mut do_update_pub = do_update_pub.lock().unwrap();
            if *do_update_pub {
                let config = config_state.lock().unwrap().clone();
                let _ = update_pub.publish(&config);
                *do_update_pub = false;
            }
        }
        // let _ = description_pub.publish(&config_description);
        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    }
}
