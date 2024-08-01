/// call a dynamic reconfigure server from the command line

use std::collections::HashMap;
use tracing_subscriber;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    let tracing_sub = tracing_subscriber::fmt().finish();
    tracing::subscriber::set_global_default(tracing_sub)?;

    let mut params = HashMap::<String, String>::new();
    params.insert("_name".to_string(), "camera_fov_plane_intersection".to_string());
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

    // TODO(lucasw) prefix ns to server name if no leading slash
    let partial_server_name = &args2[1];
    let full_server_name = &format!(
        "/{}/set_parameters",
        &partial_server_name,
        )
        .replace("//", "/")
        .replace("set_parameters/set_parameters", "set_parameters");
    let client = nh
        .service_client::<dynamic_reconfigure::Reconfigure>(&full_server_name)
        .await.expect("can't connect to {full_server_name}");
    tracing::info!("connected to dynamic reconfigure server {full_server_name}");

    let name = args2[2].to_string();

    let req = {
        let raw_value = &args2[3];
        let mut req = dynamic_reconfigure::ReconfigureRequest::default();
        // TODO(lucasw) with this setup can't set a string to 'true' or a number, need to prefix
        // it with other text to make it fail through
        // could require a type be provided on the command line instead
        if let Ok(double_value) = raw_value.parse::<f64>() {
            let mut double_param = dynamic_reconfigure::DoubleParameter::default();
            double_param.name = name;
            double_param.value = double_value;
            req.config.doubles.push(double_param);
        } else if let Ok(int_value) = raw_value.parse::<i32>() {
            let mut int_param = dynamic_reconfigure::IntParameter::default();
            int_param.name = name.to_string();
            int_param.value = int_value;
            req.config.ints.push(int_param);
        } else if let Ok(bool_value) = raw_value.parse::<bool>() {
            let mut bool_param = dynamic_reconfigure::BoolParameter::default();
            bool_param.name = name.to_string();
            bool_param.value = bool_value;
            req.config.bools.push(bool_param);
        } else {
            let mut str_param = dynamic_reconfigure::StrParameter::default();
            str_param.name = name;
            str_param.value = raw_value.to_string();
            req.config.strs.push(str_param);
        }
        // TODO(lucasw) enum params

        tracing::info!("{req:?}");
        req
    };

    let response = client.call(&req).await?;
    tracing::info!("Got response: {:?}", response);

    Ok(())
}
