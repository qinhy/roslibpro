import { RosAdvance } from "./RosAdvance.js"
export class RosSubscriber extends RosAdvance {
    createTopic() {
        this.sub = ()=>this.topic();
        const _this = this;
        this.sub().subscribe(message => {
            message._topic_type = _this.topic_type;
            this.subscribe(message);
        });
    }
    subscribe(message) {
        console.log(this.constructor.name, "subscribe not defined, need to override it");
    }
}
