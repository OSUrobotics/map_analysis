import genpy
import yaml


def extract_clusters(clusters, topics, bag_in, bag_out):
    for cluster in clusters:
        start, end = cluster

        if genpy.rostime.Time not in type(start).__bases__:
            start = genpy.rostime.Time(start)
            end = genpy.rostime.Time(end)

        bag_iterator = bag_in.read_messages(
            topics=topics,
            start_time=start,
            end_time=end
        )
        for topic, msg, stamp in bag_iterator:
            bag_out.write(topic, msg, stamp)


def get_info(bag):
    return yaml.load(bag._get_yaml_info())
