import { RosPublisher } from "./RosPublisher.js";
import { RosService } from "./RosService.js";
import { RosSubscriber } from "./RosSubscriber.js";
import { SingletonKeyValueStorage } from "./Storages.js";

export class RosClientManager extends SingletonKeyValueStorage{
    add_new_pub(rosip, topic_name, topic_type, rate){
        const uuid = `ros:pub:${this.randuuid()}`;
        this.set(uuid,new RosPublisher(rosip, topic_name, topic_type, rate))
        return uuid;
    }
    add_new_sub(rosip, topic_name, topic_type, rate){
        const uuid = `ros:sub:${this.randuuid()}`;
        this.set(uuid,new RosSubscriber(rosip, topic_name, topic_type, rate))
        return uuid;
    }
    add_new_service(rosip, topic_name, topic_type, rate){
        const uuid = `ros:srv:${this.randuuid()}`;
        this.set(uuid,new RosService(rosip, topic_name, topic_type, rate))
        return uuid;
    }
    pubs(){return this.keys('^ros:pub:*');}
    subs(){return this.keys('^ros:sub:*');}
    srvs(){return this.keys('^ros:srv:*');}
    loads(jsonStr){
        super.loads(jsonStr);
        this.pubs().forEach(k => this.set(k,new RosPublisher(this.get(k).rosip, this.get(k).topic_name, this.get(k).topic_type, this.get(k).rate)));
        this.subs().forEach(k => this.set(k,new RosSubscriber(this.get(k).rosip, this.get(k).topic_name, this.get(k).topic_type, this.get(k).rate)));
        this.srvs().forEach(k => this.set(k,new RosService(this.get(k).rosip, this.get(k).topic_name, this.get(k).topic_type, this.get(k).rate)));
    }
}