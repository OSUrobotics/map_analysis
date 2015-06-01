#!/usr/bin/env python

from mongodb_store_msgs.srv import MongoInsertMsgRequest, MongoQueryMsgRequest
import mongodb_store.util as dc_util
from datetime import datetime
from mongodb_store_msgs.msg import StringPair, StringPairList
import json
from bson import json_util
from mongodb_store.util import load_class, type_to_class_string
import rosbag
import rospy

MongoClient = dc_util.import_MongoClient()

_mongo_client=MongoClient(rospy.get_param('mongodb_host'), rospy.get_param('mongodb_port'))

def to_msg(message):
    new_msg = load_class(type_to_class_string(message._type))()
    common_attrs = set(dir(message)).intersection(dir(new_msg))
    for attr in common_attrs:
        if not attr.startswith('_') and not callable(getattr(message, attr)):
            setattr(new_msg, attr, getattr(message, attr))

def insert_msg(message, database='message_store', collection='message_store'):
    sp = (StringPair(MongoQueryMsgRequest.JSON_QUERY, json.dumps({}, default=json_util.default)),)


    # deserialize data into object
    obj = dc_util.deserialise_message(dc_util.serialise_message(message))        
    # obj = to_msg(message)
    # convert input tuple to dict
    meta = dc_util.string_pair_list_to_dictionary(StringPairList(sp))
    # get requested collection from the db, creating if necessary
    collection = _mongo_client[database][collection]
    meta['inserted_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
    meta['inserted_by'] = 'asdf'
    obj_id = dc_util.store_message(collection, obj, meta)

    return str(obj_id)

if __name__ == '__main__':
    rospy.init_node('asdf')    

    b = rosbag.Bag(open('/home/lazewatd/wheelchair_logs/04-30/compressed_wheelchair_2015-04-30-13-27-48_28.bag', 'r'))
    for _, message, _ in b.read_messages(topics=['/wheelchair_lasers/left']):
        pass

    print insert_msg(message)