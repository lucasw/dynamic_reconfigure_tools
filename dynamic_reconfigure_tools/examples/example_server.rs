/// example dynamic reconfigure server

use std::collections::HashMap;
use tracing_subscriber;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::{NodeHandle, Publisher};
    use std::sync::{Arc, Mutex};

    let tracing_sub = tracing_subscriber::fmt().finish();
    tracing::subscriber::set_global_default(tracing_sub)?;

    let mut params = HashMap::<String, String>::new();
    params.insert("_name".to_string(), "dynrec_server".to_string());
    params.insert("_ns".to_string(), "".to_string());

    // TODO(lucasw) can an existing rust arg handling library handle the ':=' ros cli args?
    let args = std::env::args();
    let mut args2 = Vec::new();
    for arg in args {
        let key_val: Vec<&str> = arg.split(":=").collect();
        if key_val.len() != 2 {
            args2.push(arg);
            continue;
        }

        let (mut key, val) = (key_val[0].to_string(), key_val[1].to_string());
        if !key.starts_with("_") {
            println!("unused arg pair {key}:={val}- need to prefix name with underscore");
            continue;
        }
        key.replace_range(0..1, "");

        if params.contains_key(&key) {
            params.insert(key, val);
        } else {
            println!("unused '{key}' '{val}'");
        }
    }
    println!("{args2:?}");

    let ns = params.remove("_ns").unwrap();
    let full_node_name = &format!(
        "/{}/{}",
        &ns,
        &params["_name"],
        ).replace("//", "/");
    let ros_master_uri = std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
    let nh = NodeHandle::new(&ros_master_uri, full_node_name).await?;
    tracing::info!("connected to roscore at {ros_master_uri}");

    // Dynamic reconfigure service and topics
    let full_server_name = &format!(
        "/{}/set_parameters",
        &full_node_name,
        )
        .replace("//", "/")
        .replace("set_parameters/set_parameters", "set_parameters");

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
    let config_pub: Publisher<dynamic_reconfigure::Config> = nh.advertise(&format!("{}/parameter_updates", full_node_name.as_str()), 3).await?;
    let description_pub: Publisher<dynamic_reconfigure::ConfigDescription> = nh.advertise(&format!("{}/parameter_descriptions", full_node_name.as_str()), 3).await?;

    let config_state_copy = config_state.clone();
    let server_fn = move |request: dynamic_reconfigure::ReconfigureRequest| {
        tracing::info!("{request:?}");

        let mut config = config_state_copy.lock().unwrap();
        tracing::info!(".");

        // TODO(lucasw) actually change values in config_state if the request
        // has matching types and names (and clip the values to min and max)
        // TODO(lucasw) need a hashmap as base structure so don't have to do double for loops
        // and then convert it to and from Config as needed
        // for intv in request.config.ints {
        // }
        'outer: for req_strv in request.config.strs {
            for strv in &mut config.strs {
                if req_strv.name == strv.name {
                    // TODO(lucasw) also need to clip to min max values
                    tracing::info!("set {} value {} to {}", strv.name, strv.value, req_strv.value);
                    strv.value = req_strv.value;
                    break 'outer
                }
            }
        }
        // TODO(lucasw) only publish if updated?
        // only do this here if latched
        // let _ = config_pub.publish(&config).await;

        Ok(dynamic_reconfigure::ReconfigureResponse {
            config: config.clone(),
        })
    };

    let _server_handle = nh
        .advertise_service::<dynamic_reconfigure::Reconfigure, _>(&full_server_name, server_fn)
        .await.expect("can't connect to {full_server_name}");
    tracing::info!("connected to dynamic reconfigure server {full_server_name}");

    // Setup a task to kill this process when ctrl_c comes in:
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0);
    });

    loop {
        // let config_state = config_state.lock().unwrap().clone();
        // tracing::info!("Current value of config: {config_state:?}");
        let config = config_state.lock().unwrap().clone();
        let _ = config_pub.publish(&config).await;
        let _ = description_pub.publish(&config_description).await;
        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    }
}
