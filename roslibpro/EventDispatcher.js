const _randuuid = (prefix = '') => {
    return prefix + 'xxxx-xxxx-xxxx-xxxx-xxxx'.replace(/x/g, function () {
        return Math.floor(Math.random() * 16).toString(16);
    });
};

class SingletonStorageController {
    exists(key) { console.log(`[${this.constructor.name}]: not implemented`); }
    set(key, value) { console.log(`[${this.constructor.name}]: not implemented`); }
    get(key) { console.log(`[${this.constructor.name}]: not implemented`); }
    delete(key) { console.log(`[${this.constructor.name}]: not implemented`); }
    keys(pattern = '*') { console.log(`[${this.constructor.name}]: not implemented`); }

    clean() { this.keys('*').forEach(k => this.delete(k)); }

    dumps() {
        const res = {};
        this.keys('*').forEach(k => res[k] = this.get(k));
        return JSON.stringify(res);
    }

    loads(jsonString = '{}') {
        this.clean();
        Object.entries(JSON.parse(jsonString)).forEach(([k, v]) => this.set(k, v));
    }

    _randuuid(prefix = '') {
        return _randuuid(prefix);
    }
}

class JavascriptDictStorage {
    constructor() {
        this.uuid = this._randuuid();
        this._store = {};
    }

    _randuuid(prefix = '') {
        return _randuuid(prefix);
    }

    store() {
        return this._store;
    }
}

class JavascriptDictStorageController extends SingletonStorageController {
    constructor(model) {
        super();
        this.__model = model;
    }

    exists(key) {
        return key in this.__model.store();
    }

    set(key, value) {
        this.__model.store()[key] = value;
    }

    get(key) {
        const store = this.__model.store();
        return (key in store) ? store[key] : null;
    }

    delete(key) {
        if (key in this.__model.store()) {
            delete this.__model.store()[key];
        }
    }

    keys(pattern = '*') {
        const escaped = pattern.replace(/[.+?^${}()|[\]\\]/g, '\\$&'); // escape regex meta
        const regex = new RegExp(escaped.replace(/\*/g, '.*'));
        return Object.keys(this.__model.store()).filter(key => regex.test(key));
    }
}

export class EventDispatcherController extends JavascriptDictStorageController {

    constructor() {
        super(new JavascriptDictStorage());
    }

    events(event_name = '*') {
        return this.keys(`Event:${event_name}:*`).map(k => [k, this.get(k)]);
    }

    get_event(uuid) {
        return this.keys(`*:${uuid}`).map(k => this.get(k));
    }

    delete_event(uuid) {
        this.keys(`*:${uuid}`).forEach(k => this.delete(k));
    }

    delete_events_by_name(event_name) {
        this.events(event_name).forEach(([k]) => this.delete(k));
    }

    set_event(event_name, callback, id = null) {
        if (id === null) id = this._randuuid();
        this.set(`Event:${event_name}:${id}`, callback);
        return id;
    }

    dispatch(event_name, ...args) {
        this.events(event_name).forEach(([_, event_func]) => {
            event_func(...args);
        });
    }
}

export class EventDispatcherControllerTest {
    constructor() {
        const dispatcher = new EventDispatcherController();

        let callback1Called = false;
        let callback2Called = false;
        let callbackArg = null;
        let eventID;

        // Test 1: set_event and get_event
        eventID = dispatcher.set_event('testEvent', () => { callback1Called = true; });
        console.assert(dispatcher.get_event(eventID).length === 1, "Test 1 Failed: Event should be set and retrievable.");
        console.assert(typeof dispatcher.get_event(eventID)[0] === 'function', "Test 1 Failed: Retrieved event should be a function.");

        // Test 2: dispatch
        dispatcher.set_event('testEvent', (arg) => { callbackArg = arg; });
        dispatcher.dispatch('testEvent', 'Hello');
        console.assert(callbackArg === 'Hello', "Test 2 Failed: Dispatch should call event with correct argument.");

        // Test 3: delete_event
        dispatcher.delete_event(eventID);
        console.assert(dispatcher.get_event(eventID).length === 0, "Test 3 Failed: Event should be deleted.");

        // Test 4: delete_events_by_name
        const eventID1 = dispatcher.set_event('deleteTest', () => { callback1Called = true; });
        const eventID2 = dispatcher.set_event('deleteTest', () => { callback2Called = true; });
        dispatcher.delete_events_by_name('deleteTest');
        console.assert(dispatcher.events('deleteTest').length === 0, "Test 4 Failed: Events by name should be deleted.");
        console.assert(!callback1Called && !callback2Called, "Test 4 Failed: Callbacks should not be called after deletion.");

        // Test 5: events
        dispatcher.set_event('anotherEvent', () => {});
        console.assert(dispatcher.events('anotherEvent').length === 1, "Test 5 Failed: Should retrieve events by name.");

        console.log("All EventDispatcherController tests completed.");
    }
}
