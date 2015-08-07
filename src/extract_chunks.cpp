#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <boost/foreach.hpp>

using namespace std;

int main(int argc, char** argv) {
    rosbag::Bag inBag, outBag;
    cout << "Opening " << argv[1] << endl;
    inBag.open(argv[1], rosbag::bagmode::Read);
    cout << "Opening " << argv[2] << endl;
    outBag.open(argv[2], rosbag::bagmode::Write);

    // create a yaml parser from cin so we can pipe yaml data in
    YAML::Parser parser(cin);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    // read in the topics
    vector<string> topics;
    doc["topics"] >> topics;

    // read in the clusters
    vector<vector<string> > clusters;
    doc["clusters"] >> clusters;

    // create a view and make sure it merges duplicate messages
    rosbag::View view(true);
    // view.addQuery(inBag, rosbag::TopicQuery(topics));

    // add time intervals to the view
    for(int i=0; i<clusters.size(); i++) {
        vector<string> vec = clusters[i];
        ros::Time start(atof(vec[0].c_str()));
        ros::Time end(atof(vec[1].c_str()));
        view.addQuery(inBag, start, end);
    }

    // write all messages from the view to the new bag
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        outBag.write(m.getTopic(), m.getTime(), m);
    }

    inBag.close();
    outBag.close();

    return 0;
}