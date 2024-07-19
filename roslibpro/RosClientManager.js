import { RosPublisher } from "./RosPublisher.js";
import { RosService } from "./RosService.js";
import { RosSubscriber } from "./RosSubscriber.js";
import { SingletonKeyValueStorage } from "./Storages.js";

export class RosClientManager extends SingletonKeyValueStorage{
    _uuid2key(uuid){
        return uuid.split(':')[1];
    }
    _key2type(key){
        const _instance_map = {
            'pub':RosPublisher,'sub':RosSubscriber,'srv':RosService,}        
        return _instance_map[key];
    }
    _set_instance(key, rosip, topic_name, topic_type, rate, uuid=null){
        if(!uuid)uuid = `ros:${key}:${this.randuuid()}`;
        this.set(uuid,new (this._key2type(key))(rosip, topic_name, topic_type, rate))
        return uuid;
    }
    add_new_pub(rosip, topic_name, topic_type, rate){
        return this._set_instance('pub', rosip, topic_name, topic_type, rate);
    }
    add_new_sub(rosip, topic_name, topic_type, rate){
        return this._set_instance('sub', rosip, topic_name, topic_type, rate);
    }
    add_new_service(rosip, topic_name, topic_type, rate){
        return this._set_instance('srv', rosip, topic_name, topic_type, rate);
    }
    all_keys(){        
        return this.keys('^ros:*');
    }
    all_instances(){        
        return this.all_keys().map(k=>this.get(k));
    }
    connect(){
        this.all_instances().forEach(c => c.connectROS());
    }
    close(){
        this.all_instances().forEach(c => c.close());
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
    }
    
    pubs(){return this.keys('^ros:pub:*').map(k=>this.get(k));}
    subs(){return this.keys('^ros:sub:*').map(k=>this.get(k));}
    srvs(){return this.keys('^ros:srv:*').map(k=>this.get(k));}

    loads(jsonStr,rosip=null){
        super.loads(jsonStr);
        const allmember = this.all_keys().map(uuid => {return {uuid, ...this.get(uuid)}});
        allmember.forEach(m => this._set_instance(this._uuid2key(m.uuid),rosip??m.rosip, m.topic_name, m.topic_type, m.rate, m.uuid));
    }
}