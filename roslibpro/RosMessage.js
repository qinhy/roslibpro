
import { RosPublisher } from "./RosPublisher.js";
import { RosService } from "./RosService.js";
import { RosSubscriber } from "./RosSubscriber.js";
import { SingletonKeyValueStorage } from "./Storages.js";


class RosClientManager extends SingletonKeyValueStorage{
    _uuid2type(uuid){
        return uuid.split(':')[1];
    }
    _type2class(key){
        return{'pub':RosPublisher,
               'sub':RosSubscriber,
               'srv':RosService}[key];
    }
    _set_instance(key, rosip, topic_name, topic_type, rate=10, uuid=null){
        if(!uuid)uuid = `ros:${key}:${this.randuuid()}`;
        this.set(uuid,new (this._type2class(key))(rosip, topic_name, topic_type, rate))
        return uuid;
    }
    add_new_pub(rosip, topic_name, topic_type, rate=10){
        return this._set_instance('pub', rosip, topic_name, topic_type, rate);
    }
    add_new_sub(rosip, topic_name, topic_type, rate=10){
        return this._set_instance('sub', rosip, topic_name, topic_type, rate);
    }
    add_new_service(rosip, topic_name, topic_type, rate=10){
        return this._set_instance('srv', rosip, topic_name, topic_type, rate);
    }
    all_keys(){        
        return this.keys('^ros:*');
    }
    all_instances(){        
        return this.all_keys().map(k=>this.get(k));
    }
    delete(key){
        this.get(key).close();
        super.delete(key);
    }
    connect(){
        this.all_instances().forEach(c => {if(c.connectROS)c.connectROS()});
    }
    close(){
        this.all_instances().forEach(c => {if(c.close)c.close()});
    }
    clean(){
        this.close();
        super.clean();
    }
    // change_all_ip(rosip){
    //     const data = this.dumps();
    //     this.clean();
    //     this.loads(data,rosip);
    //     this.connect();
    //     return this;
    // }
    
    pub_keys(){return this.keys('^ros:pub:*');}
    sub_keys(){return this.keys('^ros:sub:*');}
    srv_keys(){return this.keys('^ros:srv:*');}
    
    pubs(){return this.pub_keys().map(k=>this.get(k));}
    subs(){return this.sub_keys().map(k=>this.get(k));}
    srvs(){return this.srv_keys().map(k=>this.get(k));}

    // loads(jsonStr,rosip=null){
    //     super.loads(jsonStr);
    //     this.all_keys().map(uuid => {return {uuid, ...this.get(uuid)}})
    //         .forEach(m => this._set_instance(this._uuid2type(m.uuid),rosip??m.rosip,
    //                                     m.topic_name, m.topic_type, m.rate, m.uuid));
    // }
    
    _listener_uuid_2_parent_uuid(uuid){
        return uuid.replace('_listeners','').replace(':'+uuid.split(':').pop(),'');
    }
    _listener_uuid(uuid){
        const type = this._uuid2type(uuid)
        return uuid.replace(type,type+'_listeners')
    }
    _set_new_sub_listener(sub_uuid,listener_fun){
        const listener_uuid = `${this._listener_uuid(sub_uuid)}:${this.randuuid()}`;
        this.set(listener_uuid,listener_fun);
        return listener_uuid;
    }
    _first_sub_uuid(topic_name){
        const uuid = this.sub_keys().filter(s=>this.get(s).topic_name==topic_name)[0];
        return uuid;
    }
    get_sub_listener_keys(topic_name){
        var k = this._first_sub_uuid(topic_name);
        return k?this.keys(`${this._listener_uuid(k)}:*`):[];
    }
    add_sub_listener(rosip, topic_name, topic_type, listener_fun){
        if(!this._first_sub_uuid(topic_name)){
            const uuid = this._add_new_sub(rosip, topic_name, topic_type);                
            this.get(uuid).connectROS({
                onError: (e) => {console.log(e)},
                onConnection: () => {this._set_new_sub_listener(uuid,listener_fun)},
                onClose: () => {}
            });  
        }
    }
    delete_sub_listener(uuid){
        const sub_uuid = this._listener_uuid_2_parent_uuid(uuid);
        const topic_name = this.get(sub_uuid).topic_name;
        this.delete(uuid);
        if (this.get_sub_listener_keys(topic_name).length==0){
          this.get(sub_uuid).close();
          this.delete(sub_uuid);
        }
    }
}


class RosAbstractModel{// extends RosClientManager{
    constructor(empty=false) {
        // super();
        this.___pubsub = 'pub';
        if(empty)return;

        // Publishers
        // BH -> UI, 1000ms, std_msgs/msg/Int32
        // this.app_state = null//std_msgs.msg.Int32();s

        // Subscribers
        // UI -> BH, XXms, std_msgs/msg/Float32MultiArray
        // this._operation_cmd = null//std_msgs.msg.Float32MultiArray();
    }

    // service topic with "___"
    // ___some_service(){}

    
    _is_private(param_name) {
        return param_name.includes('__') || param_name.includes('/__');
    }

    _is_sub(param_name) {
        return param_name.startsWith('_') || param_name.includes('/_');
    }

    _get_pubs_path() {
        const name = this.constructor.name;
        const params = this._dfs(this);
        return params.filter(([isFunc, n, v]) => !isFunc && !this._is_sub(n)).map(([isFunc, i,v]) => `${name}/${i}  :  ${v.getTopicType()}`);
    }

    _get_subs_path() {
        const name = this.constructor.name;
        const params = this._dfs(this);
        return params.filter(([isFunc, n, v]) => !isFunc && this._is_sub(n)).map(([isFunc, i,v]) => `${name}/${i}  :  ${v.getTopicType()}`);
    }

    _get_srvs_path() {
        const name = this.constructor.name;
        return this._dfs(this).filter(([isFunc]) => isFunc).map(([isFunc, n]) => `${name}/${n}`);
    }

    _is_function(param) {
        return typeof param === 'function';
    }

    _is_primitive(param) {
        return param === null || this._is_function(param) || param.hasOwnProperty('__empty');
    }

    _dfs(param, path = '', isFunc = false, onlyFunc = false) {
        const paths = [];

        if (this._is_primitive(param)) {
            paths.push([this._is_function(param), path, param]);
        } else {
            for (const [key, value] of Object.entries(param)) {
                if (this._is_private(key)) continue;
                const isFunction = this._is_function(value);
                if (isFunction && key.startsWith('_')) continue;
                const newPath = `${path}/${key}`;
                paths.push(...this._dfs(value, newPath, isFunction, onlyFunc));
            }
        }
        return paths;
    }

    _isPubModel() {
        return this.___pubsub == 'pub';
    }

    _isPrivate(paramName) {
        return paramName.includes('__') || paramName.includes('/__');
    }

    _isSub(paramName) {
        return paramName.startsWith('_') || paramName.includes('/_');
    }

    _isPrimitive(obj) {
        return !obj.hasOwnProperty('___pubsub');
    }

    _pubs() {
        const menbers = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && this._isPrimitive(key) && !this._isSub(key) && !this._isPrivate(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _subs() {
        const menbers = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && this._isPrimitive(key) && this._isSub(key) && !this._isPrivate(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _to_subs(obj) {
        obj = {...obj};
        for (const key in obj) {
            if (obj.hasOwnProperty(key) && !key.startsWith('_')) {
                const newKey = `_${key}`; // Add "_" to the beginning
                obj[newKey] = obj[key];   // Assign the value to the new key
                delete obj[key];          // Delete the old key
            }
        }
        return obj;
    }
    _to_pubs(obj) {
        obj = {...obj};
        for (const key in obj) {
            if (obj.hasOwnProperty(key) && key.startsWith('_')) {
                const newKey = key.slice(1); // Remove the first character
                obj[newKey] = obj[key];      // Assign the value to the new key
                delete obj[key];             // Delete the old key
            }
        }
        return obj;
    }

    _toPubModel() {
        var res = this._isPubModel()?this:this._switchModel();
        res.___pubsub = 'pub';
        return res;
    }

    _toSubModel() {
        var res = this._isPubModel()?this._switchModel():this;
        res.___pubsub = 'sub';
        return res;
    }
    
    _switchModel() {
        const instance = new this.constructor(true);
        const ispub = this._isPubModel();
        for (const key in this) {
            if (!this._isPrimitive(this[key])) {
                if(this[key]._isPubModel()){
                    instance[`_${key}`] = this[key]._toSubModel();
                }
                else{
                    instance[key.slice(1)] = this[key]._toPubModel();
                }                
            }
        }
        // console.log(instance);
        Object.assign(instance, this._to_subs(this._pubs()));
        Object.assign(instance, this._to_pubs(this._subs()));
        instance.___pubsub = ispub?'sub':'pub';
        return instance
    }

    _randomSetMembers() {
        for (const key in this) {
            if(this._isPrivate(key))continue;
            if(this[key].hasOwnProperty('__empty')){
                this[key].randomSetMembers();
            }
            else{
                this[key]._randomSetMembers();
            }
        }
    }

}


/// ros messages 
class RosMessageBase {
    constructor() {
        this.__empty = true;
    }
    
    empty() { return this.__empty; }

    toRosBridgeFormat() {
        const publicData = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && !key.startsWith('__')) {
                publicData[key] = this[key];
            }
        }
        return publicData;
    }

    fromRosBridgeFormat(data) {
        const instance = new this.constructor();
        obj = Object.assign(instance, data);
        this.__empty = false;        
        Object.assign(this, obj);
        return this;
    }

    getTopicType() {
        return this.constructor.name.replace(/__/g, '/');
    }
    
    _isPrivate(paramName) {
        return paramName.includes('__') || paramName.includes('/__');
    }

    randomSetMembers() {
        const dfsRandomSet = (obj) => {
            for (let key in obj) {
                if ( !this._isPrivate(key) && typeof obj[key] === 'object' && obj[key] !== null) {
                    // Recursively set values for nested objects
                    dfsRandomSet(obj[key]);
                } else if (typeof obj[key] === 'number') {
                    // Set a random number for numeric properties
                    obj[key] = Math.random() * 10; // Adjust range as needed
                } else if (typeof obj[key] === 'string') {
                    // Set a random string for string properties
                    obj[key] = Math.random().toString(36).substring(2, 8); // Generates a random 6-character string
                } else if (typeof obj[key] === 'boolean') {
                    // Set a random boolean value
                    obj[key] = Math.random() < 0.5; // Randomly sets to true or false
                }
            }
        };
        dfsRandomSet(this);
    }
}

class nav_msgs__msg__Odometry extends RosMessageBase {
    constructor() {
        super();
        this.header = {seq:0,time:"20241010T00:00:00",frame_id:"null"};
        this.child_frame_id = "null";
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        this.twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    }
}

class sensor_msgs__msg__JointState extends RosMessageBase {
    constructor() {
        super();
        this.header = {seq:0,time:"20241010T00:00:00",frame_id:"null"};
        this.name = [];
        this.position = [];
        this.velocity = [];
        this.effort = [];
    }
}

class std_msgs__msg__Int8MultiArray extends RosMessageBase {
    constructor() {
        super();
        this.layout = { dim: [], data_offset: 0 };
        this.data = [];
    }
}

class std_msgs__msg__Float32MultiArray extends RosMessageBase {
    constructor() {
        super();
        this.layout = {
            dim: [], // array of dimensions, each with label, size, and stride
            data_offset: 0 // offset for the data in the array
        };
        this.data = []; // array of float32 values
    }
}

class rdt_interfaces__msg__ApplicationState extends RosMessageBase {
    constructor() {
        super();
        this.state = '';
        this.details = '';
    }
}

class visualization_msgs__msg__MarkerArray extends RosMessageBase {
    constructor() {
        super();
        this.markers = [];
    }

    fromRosBridgeFormat(data) {
        const instance = new this.constructor();
        instance.markers = data.markers.map(markerData => visualization_msgs__msg__Marker().fromRosBridgeFormat(markerData));
        return instance;
    }
}

class visualization_msgs__msg__Marker extends RosMessageBase {
    constructor() {
        super();
        this.header = {seq:0,time:"20241010T00:00:00",frame_id:"null"};
        this.ns = '';
        this.id = 0;
        this.type = 0;
        this.action = 0;
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        this.scale = { x: 1, y: 1, z: 1 };
        this.color = { r: 1.0, g: 1.0, b: 1.0, a: 1.0 };
        this.lifetime = 0;
        this.points = [];
        this.text = '';
    }
}

class nav_msgs__msg__Path extends RosMessageBase {
    constructor() {
        super();
        this.header = {seq:0,time:"20241010T00:00:00",frame_id:"null"};
        this.poses = [];
    }

    fromRosBridgeFormat(data) {
        const instance = new this.constructor();
        instance.header = data.header;
        instance.poses = data.poses.map(poseData => this.fromRosBridgeFormat(poseData));
        return instance;
    }
}

class geometry_msgs__msg__PoseStamped extends RosMessageBase {
    constructor() {
        super();
        this.header = {seq:0,time:"20241010T00:00:00",frame_id:"null"};
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
    }
}

class std_msgs__msg__Bool extends RosMessageBase {
    constructor() {
        super();
        this.data = false;
    }
}

class std_msgs__msg__String extends RosMessageBase {
    constructor(data = '') {
        super();
        this.data = data;
    }
}

class std_msgs__msg__Int32 extends RosMessageBase {
    constructor(data = 0) {
        super();
        this.data = data;
    }
}

const sensor_msgs = { msg: {} };
const rdt_interfaces = { msg: {} };
const visualization_msgs = { msg: {} };
const nav_msgs = { msg: {} };
const geometry_msgs = { msg: {} };
const std_msgs = { msg: {} };

nav_msgs.msg.Odometry = nav_msgs__msg__Odometry;
sensor_msgs.msg.JointState = sensor_msgs__msg__JointState;
std_msgs.msg.Int8MultiArray = std_msgs__msg__Int8MultiArray;
std_msgs.msg.Float32MultiArray = std_msgs__msg__Float32MultiArray;
rdt_interfaces.msg.ApplicationState = rdt_interfaces__msg__ApplicationState;
visualization_msgs.msg.MarkerArray = visualization_msgs__msg__MarkerArray;
visualization_msgs.msg.Marker = visualization_msgs__msg__Marker;
nav_msgs.msg.Path = nav_msgs__msg__Path;
geometry_msgs.msg.PoseStamped = geometry_msgs__msg__PoseStamped;
std_msgs.msg.Bool = std_msgs__msg__Bool;
std_msgs.msg.String = std_msgs__msg__String;
std_msgs.msg.Int32 = std_msgs__msg__Int32;


class RosAbstractBase {
}

// class TestRosAbstractBase extends RosAbstractBase {
//     constructor() {
//         super();
//         this.name = 'It is a test.';
//         this.time = 154541352;
//         this._weather = null;
//     }

//     who_I_am() {
//         return `${this.name} ${this.time}`;
//     }
// }

class RosRoot extends RosAbstractModel {
    constructor() {
        super();
        // this.test = new TestRosAbstractBase();
    }

    _get_srvs() {
        const name = this.constructor.name;
        const fs = super._get_srvs();
        return fs.filter(f => ![
            `${name}//sub_topic`,
            `${name}//call_service`,
            `${name}//all_list`,
            `${name}//service_list`,
            `${name}//topic_list`
        ].includes(f));
    }

    sub_topic(path, ...args) {
        if (this._is_sub(path)) return 'Cannot sub a "try Sub" topic';
        let param = this;
        for (const i of path.split('/')) {
            if (i.length === 0) continue;
            param = param[i];
            if (!this._is_function(param) && this._is_primitive(param)) {
                return param;
            }
        }
        return null;
    }

    call_service(path, ...args) {
        let param = this;
        for (const i of path.split('/')) {
            if (i.length === 0) continue;
            param = param[i];
            if (this._is_function(param)) {
                return param(...args);
            }
        }
        return null;
    }

    all_list() {
        let res = '\n----------------------------------------';
        res += '\npub topics:\n';
        res += this._get_pubs_path().map(pub => pub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsub topics:\n';
        res += this._get_subs_path().map(sub => sub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsrv topics:\n';
        res += this._get_srvs_path().map(srv => srv.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }

    service_list() {
        let res = '\n----------------------------------------';
        res += '\nsrv topics:\n';
        res += this._get_srvs_path().map(srv => srv.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }

    topic_list() {
        let res = '\n----------------------------------------';
        res += '\npub topics:\n';
        res += this._get_pubs_path().map(pub => pub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsub topics:\n';
        res += this._get_subs_path().map(sub => sub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }
}

// const root = new RosRoot();
// console.log(root.all_list());
// console.log(root.call_service('/test/who_I_am'));
// console.log(root.sub_topic('/test/name'));


// tests 

// class VisModel extends RosAbstractModel{
//     constructor() {
//         super();
//         // Publishers
//         // BH -> UI, 1000ms, std_msgs/msg/Int32
//         this.app_state = new visualization_msgs.msg.Marker();

//     }
// }
// class AppModel extends RosAbstractModel{
//     constructor() {
//         super();
//         // Publishers
//         // BH -> UI, 1000ms, std_msgs/msg/Int32
//         this.app_state = new std_msgs.msg.Int32();

//         // Subscribers
//         // UI -> BH, XXms, std_msgs/msg/Float32MultiArray
//         this._operation_cmd = new std_msgs.msg.Float32MultiArray();
//         this._vis = new VisModel();

//     }
// }

// app = new AppModel();

export { sensor_msgs, rdt_interfaces, visualization_msgs, nav_msgs, geometry_msgs, std_msgs , RosAbstractModel, RosRoot}