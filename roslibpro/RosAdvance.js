import { RosBase } from "./RosBase.js";

export class RosAdvance extends RosBase {
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false) {
        super(rosip, topic_name, topic_type, rate, is_service);
        this._keepConnection_running = false;
    }

    connectROS(callback = { onError: (error) => { }, onConnection: () => { }, onClose: () => { } },
        auto_retry = true, call_from_outter = true) {
        this.callback = ()=>callback;
        if (!super.connectROS(callback, call_from_outter)) return;
        this.createTopic();
        if (!this._keepConnection_running && auto_retry) {
            this._keepConnection();
        }
    }

    _keepConnection(interval = 2000) {
        this._keepConnection_running = true;
        this._keepConnection_intervalID = setInterval(() => {
            if (this.self_closed && this.isNULL() && this.connected_cnt > 0) return;
            if (this.isConnectDone() || this.isConnecting() || this.isClosing()) return;
            if (!this.self_closed && this.isClosed() && this.connected_cnt > 0) {
                super.connectROS(this.callback(), this.rate, false, false, this.is_service);
                this.createTopic();
            }
        }, interval);
    }

    callService(param = {}, timeout = 1000) {
        console.log(this.constructor.name, "callService not defined, need to override it");
    }

    createTopic() {
        // Only for subscribe or publish, Implement topic creation logic here if needed
    }
    subscribe(message) {
        console.log(this.constructor.name, "subscribe not defined, need to override it");
    }
    publish(message, logging = false) {
        console.log(this.constructor.name, "publish not defined, need to override it");
    }

    destroy() {
        this.close();
        this._keepConnection_running = false;
        clearInterval(this._keepConnection_intervalID);
    }

    close(is_self_closed = true) {
        super.close(is_self_closed);
    }
}

export class RosPublisher extends RosAdvance {
    constructor(rosip, topic_name, topic_type, rate = 10) {
        super(rosip, topic_name, topic_type, rate, false);
    }
    createTopic() {
        this.pub = ()=>this.topic();
    }
    publish(message) {
        // console.log(message);
        this.pub().publish(new ROSLIB.Message(message));
    }
}

export class RosService extends RosAdvance {
    constructor(rosip, topic_name, topic_type, rate = 10) {
        super(rosip, topic_name, topic_type, rate, true);
    }
    callService(param = {}, timeout = 1000) {
        if(!this.topic()){
            throw Error('You must call connectROS(...) at first!')
        }
        const client = this.topic();
        const _this = this;
        // this.service_response = null;
        return new Promise((resolve, reject) => {
            try {
                client.callService(
                    new ROSLIB.ServiceRequest(param), 
                    function (result) {
                        _this.service_response = result;
                        resolve(result);
                });
            } catch (err) {
                reject(err);
            }
        });

    }
}

export class RosSubscriber extends RosAdvance {
    constructor(rosip, topic_name, topic_type, rate = 10) {
        super(rosip, topic_name, topic_type, rate, false);
    }
    createTopic() {
        this.sub = ()=>this.topic();
        const _this = this;
        this.sub().subscribe(message => {
            message._topic_type = _this.topic_type;
            this.subscribe(message);
        });
    }
    subscribe(message) {
        console.error(this.constructor.name, "function of subscribe not defined, need to override it");
    }
}
