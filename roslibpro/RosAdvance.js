import { RosBase } from "./RosBase.js";

export class RosAdvance extends RosBase {
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false) {
        super(rosip, topic_name, topic_type, rate, is_service);
        this._keepConnection_running = false;
    }

    connectROS(callback = { onError: (error) => { }, onConnection: () => { }, onClose: () => { } },
        rate = null, auto_retry = true, call_from_outter = true) {
        this.callback = callback;
        this.rate = rate;
        if (!super.connectROS(callback, auto_retry, call_from_outter)) return;
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
                super.connectROS(this.callback, this.rate, false, false, this.is_service);
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
        ['pub', 'client', 'sub']
            .filter(r => this[r])
            .forEach(r => {
                this[r].unadvertise();
                this[r] = null;
            });
        super.close(is_self_closed);
    }
}
