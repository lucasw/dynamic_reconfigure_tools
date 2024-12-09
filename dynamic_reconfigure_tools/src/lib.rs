use roslibrust_util::dynamic_reconfigure;

pub fn get_server_name(partial_server_name: &str) -> String {
    format!("/{}/set_parameters", &partial_server_name,)
        .replace("//", "/")
        .replace("set_parameters/set_parameters", "set_parameters")
}

#[derive(Debug)]
pub enum ParamType {
    INT,
    DOUBLE,
    BOOL,
    STR,
}

pub fn raw_value_to_request(
    raw_value: &str,
    name: &str,
) -> (dynamic_reconfigure::ReconfigureRequest, ParamType) {
    let param_type;

    let name = name.to_string();
    let mut req = dynamic_reconfigure::ReconfigureRequest::default();
    // TODO(lucasw) with this setup can't set a string to 'true' or a number, need to prefix
    // it with other text to make it fail through
    // could require a type be provided on the command line instead
    if let Ok(value) = raw_value.parse::<i32>() {
        let int_param = dynamic_reconfigure::IntParameter { name, value };
        req.config.ints.push(int_param);
        param_type = ParamType::INT;
    } else if let Ok(value) = raw_value.parse::<f64>() {
        let double_param = dynamic_reconfigure::DoubleParameter { name, value };
        req.config.doubles.push(double_param);
        param_type = ParamType::DOUBLE;
    } else if let Ok(value) = raw_value.parse::<i32>() {
        let int_param = dynamic_reconfigure::IntParameter { name, value };
        req.config.ints.push(int_param);
        param_type = ParamType::INT;
    } else if let Ok(value) = raw_value.parse::<bool>() {
        let bool_param = dynamic_reconfigure::BoolParameter { name, value };
        req.config.bools.push(bool_param);
        param_type = ParamType::BOOL;
    } else {
        let value = raw_value.to_string();
        let str_param = dynamic_reconfigure::StrParameter { name, value };
        req.config.strs.push(str_param);
        param_type = ParamType::STR;
    }
    // TODO(lucasw) enum params

    tracing::info!("{req:?}");
    (req, param_type)
}
