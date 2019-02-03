#!/usr/bin/env node

'use strict'

global.XMLHttpRequest = require("xhr2");
const assert = require("assert");
// Main requirements
const tf = require('@tensorflow/tfjs');
const rosnodejs = require('rosnodejs');
const stringify = require('json-stringify');
// Requires the std_msgs message and sensor_msgs packages
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
const StringMsg = rosnodejs.require('std_msgs').msg.String;


async function run() {
    const rosNode = await rosnodejs.initNode('posenet')
    // ROS function for simple recieveing node param
    const getParam = async function(key, default_value){
        if(await rosNode.hasParam(key)){
            const param = await rosNode.getParam(key)
            return param
        }
        return default_value
    }
    // Find if GPU is enabled and start tf
    const gpu = await getParam('gpu', false)
    console.log(gpu)
    if (gpu)
        require('@tensorflow/tfjs-node-gpu');
    else
        require('@tensorflow/tfjs-node');
    const posenet = require('@tensorflow-models/posenet')
    // lowest quality first
    const multiplier = await getParam('multiplier', 0.5)

    const net  = await posenet.load(multiplier);
    // Local variables for sync with ROS
    let buffer = []
    let newBuffer = false
    let image_width = 0
    let image_height = 0
    let header = null
    // Parameters for posenet
    let enabled = await getParam('/posenet_enabled', true);
    const imageScaleFactor = await getParam('image_scale_factor', 0.5);
    const flipHorizontal = await getParam('flip_horizontal', false);
    const outputStride = await getParam('output_stride', 16);
    const maxPoseDetections = await getParam('max_pose', 5);
    const scoreThreshold = await getParam('score_threshold', 0.5);
    const nmsRadius = await getParam('nms_radius', 20);
    // topic names
    const camera_topic = await getParam('topic','/openni2/color')
    const output_topic = await getParam('poses_topic','js_poses')
    // ROS topics
    let pub = rosNode.advertise(output_topic, StringMsg)


    let sub = rosNode.subscribe(camera_topic, sensor_msgs.Image,
        (data) => {
            // TODO more encodings
            // Currently works wonly with rgb8 data
            assert(data.encoding == 'bgr8')
            header = data.header
            let delay = Math.floor(Date.now()) / 1000 - header.stamp.secs - header.stamp.nsecs/1000000000
            //if (delay > 0.5) return
            console.log("debug")
            buffer = data.data
            newBuffer = true
            image_height = data.height
            image_width = data.width
            //DetectingPoses()
        }
    );
    // Loop for detecting poses
    let singleInstance = false
    const DetectingPoses = async function (){
        if (!enabled){
            return
        }
        if (newBuffer == false)  return
        if (singleInstance){
            console.log("Skip")
            return
        }
        singleInstance = true
        let ts = header.stamp
        let tensor = tf.tensor3d(buffer, [image_height,image_width,3], 'int32')
        newBuffer = false
        const poses = await net.estimateMultiplePoses(tensor, imageScaleFactor, flipHorizontal, outputStride,
                                                       maxPoseDetections, scoreThreshold,nmsRadius);
        tensor.dispose();
        let msg = {ts: ts, poses:poses}
        pub.publish({data: stringify(msg)})
        singleInstance = false
    }
    setInterval(DetectingPoses, 50)
    // Check if posenet is not paused
    const checkEnabled = async function(){
        enabled = await getParam('/posenet_enabled', true);
    }
    setInterval(checkEnabled, 3000)

}





run();
