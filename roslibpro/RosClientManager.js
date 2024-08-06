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
    change_all_ip(rosip){
        const data = this.dumps();
        this.clean();
        this.loads(data,rosip);
        this.connect();
        return this;
    }
    
    pub_keys(){return this.keys('^ros:pub:*');}
    sub_keys(){return this.keys('^ros:sub:*');}
    srv_keys(){return this.keys('^ros:srv:*');}
    
    pubs(){return this.pub_keys().map(k=>this.get(k));}
    subs(){return this.sub_keys().map(k=>this.get(k));}
    srvs(){return this.srv_keys().map(k=>this.get(k));}

    loads(jsonStr,rosip=null){
        super.loads(jsonStr);
        this.all_keys().map(uuid => {return {uuid, ...this.get(uuid)}})
            .forEach(m => this._set_instance(this._uuid2type(m.uuid),rosip??m.rosip,
                                        m.topic_name, m.topic_type, m.rate, m.uuid));
    }
}


class RosBridgeSubManagerModel {
    constructor(rosip='localhost:9090'){
        //topic_name : [sub_listener ... ]
        this.rosip = rosip;
        this.root_srv_topic_name='/rosapi/topics';
        this.root_srv_topic_type='rosapi/Topics';
        this.sub_listeners = {}
    }
    get_root_srv(rate=10){
        return new RosService(this.rosip,this.root_srv_topic_name,this.root_srv_topic_type,rate)
    }
    get_topic_names(){
        return Object.entries(this.sub_listeners).map(
            e=>{
                topic_name=e[0];
                listener_fun=e[1];
                return topic_name;
            }
        )

    }

}
class RosBridgeSubManagerController {
    constructor(model=new RosBridgeSubManagerModel()){
        this.model = model
        //topic_name : client
        this._sub_clients={}
        this._root_srv=this.model.get_root_srv();
    }
    get_root_srv(){
        if(!this._root_srv.isConnectDone()){
            return new Promise((resolve, reject) => {
                    this._root_srv.connectROS(
                        {
                            onError: (e) => {reject(e)},
                            onConnection: () => resolve(this._root_srv),
                            onClose: () => {}
                        },false)
            });
        }
        else{            
            return new Promise((resolve, reject) => {resolve(this._root_srv)});
        }
    }
    get_topics(){        
        return new Promise((resolve, reject) => {
            this.get_root_srv().then(root_srv => {
                root_srv.callService().then((result) => {
                    resolve(result);
                }).catch((err) => {                
                    reject(err);console.log(err);
                });
            });
        });
    }
    get_topic_type(topic_name){
        return new Promise((resolve, reject) => {
            this.get_topics().then(result=>{
                if(result.topics && result.topics.includes(topic_name)){
                    const topic_type = result.types[result.topics.indexOf(topic_name)];
                    resolve(topic_type);
                }
                else{
                    throw Error(`not such topic of ${topic_name}`)
                }
            }).catch(e=>reject(e))
        });
    }
    add_sub_listener(topic_name, listener_fun){
        if(!this.model.sub_listeners[topic_name]){
            this.model.sub_listeners[topic_name]=[];
        }
        this.model.sub_listeners[topic_name].push(listener_fun);

        if(!this._sub_clients[topic_name]){
            this.get_topic_type(topic_name).then(topic_type=>{                
                this._sub_clients[topic_name]=new RosSubscriber(
                    this.model.rosip,topic_type,10
                );
                this._sub_clients[topic_name].connectROS(
                    {   onError: (e) => {//delete this sub
                        },
                        onConnection:() => {
                            this._sub_clients[topic_name].subscribe = (msg)=>{
                                this.model.sub_listeners[topic_name].forEach(
                                    f => f(msg)
                                )
                            }
                        },
                        onClose: () => {}
                    },false)
            });
        }

        
    }

}
class RosBridgeManagerState {
    _states = {
        WaitConnection:'WaitConnection',
    }
    _transitions = {
        
    }
}
class RosBridgeManager extends RosClientManager{
    constructor(rosip){
        if(!rosip){
            throw Error('please input ros ip')
        }
        super();
        this.init(rosip,false);
    }
    init(rosip,auto_retry = true){
        this.onConnection = () => {};
        this.root_topics_srv_uuid = `ros:srv:root`;
        if(!this.get(this.root_topics_srv_uuid)){
            this._set_instance('srv', rosip, '/rosapi/topics', 'rosapi/Topics', 10, this.root_topics_srv_uuid);
            this.get(this.root_topics_srv_uuid).connectROS({
                onError: (e) => {},
                onConnection: () => {this.onConnection()},
                onClose: () => {}
            },auto_retry)
        }
        // this.root_services_srv_uuid = super.add_new_service(rosip,'/rosapi/services', 'rosapi/ServiceType');
        // this.get(this.root_services_srv_uuid).connectROS()
    }
    _root_conn(){
        return this.get(this.root_topics_srv_uuid) && this.get(this.root_topics_srv_uuid).isConnectDone();
    }
    _test_conn(){        
        if(!this._root_conn()){
            throw Error('can not connect bridge!')
        }
    }
    get_rosip(){
        this._test_conn();
        return this.get(this.root_topics_srv_uuid).rosip
    }
    get_topics(){
        this._test_conn();
        return new Promise((resolve, reject) => {
            this.get(this.root_topics_srv_uuid).callService().then((result) => {
                resolve(result);
            }).catch((err) => {                
                reject(err);console.log(err);
            });
        });
    }
    get_services(){
        this._test_conn();
        return new Promise((resolve, reject) => {
            this.get(this.root_services_srv_uuid).callService().then((result) => {
                resolve(result);
            }).catch((err) => {                
                reject(err);console.log(err);
            });
        });
    }
    get_topic_type(topic_name){
        this._test_conn();
        return new Promise((resolve, reject) => {
            this.get_topics().then(result=>{
                if(result.topics && result.topics.includes(topic_name)){
                    const topic_type = result.types[result.topics.indexOf(topic_name)];
                    resolve(topic_type);
                }
                else{
                    throw Error(`not such topic of ${topic_name}`)
                }
            }).catch(e=>reject(e))
        });
    }

    add_new_pub(topic_name, rate=10){
        this._test_conn();
        return new Promise((resolve, reject) => {
            this.get_topic_type(topic_name).then(topic_type=>{
                const pub_uuid = this._set_instance('pub', this.get_rosip(), topic_name, topic_type, rate);
                resolve(pub_uuid);
            }).catch(e=>reject(e))
        })
    }
    add_new_sub(topic_name, rate=10){
        this._test_conn();
        return new Promise((resolve, reject) => {
            this.get_topics().then(result=>{
                console.log(result)
                if(result.topics && result.topics.includes(topic_name)){
                    const topic_type = result.types[result.topics.indexOf(topic_name)];
                    const uuid = super.add_new_sub(this.get_rosip(),topic_name,topic_type,rate)
                    this.get(uuid).subscribe = (msg)=>{this.get_sub_listener_keys(topic_name).forEach(l=>this.get(l)(msg))};
                    resolve(uuid);
                }
                else{
                    throw Error(`not such topic of ${topic_name}`)
                }
            }).catch(e=>reject(e))
        });
    }
    _first_sub_uuid(topic_name){
        const uuid = this.sub_keys().filter(s=>this.get(s).topic_name==topic_name)[0];
        return uuid;
    }
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
    get_sub_listener_keys(topic_name){
        this._test_conn();
        var k = this._first_sub_uuid(topic_name);
        return k?this.keys(`${this._listener_uuid(k)}:*`):[];
    }
    add_sub_listener(topic_name, listener_fun){     
        this._test_conn();   
        return new Promise((resolve, reject) => {
            if(!this._first_sub_uuid(topic_name)){
                this.add_new_sub(topic_name).then(uuid=>{              
                    this.get(uuid).connectROS({
                        onError: (e) => {reject(e)},
                        onConnection: () => {resolve(this._set_new_sub_listener(uuid,listener_fun));},
                        onClose: () => {}
                    });      
                    
                }).catch(e=>reject(e));
            }
            else{
                resolve(this._set_new_sub_listener(this._first_sub_uuid(topic_name),listener_fun));
            }
        })
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
    add_new_service(topic_name, topic_type, rate=10){
        this._test_conn();
        return this._set_instance('srv', this.get_rosip(), topic_name, topic_type, rate);
    }

}
export {RosClientManager,RosBridgeManager} 