#!/usr/bin/env python2

""" 
    Copyright 2018-03-02 Alberto Hata
    
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    
    http://www.apache.org/licenses/LICENSE-2.0
    
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

import os
import shutil
import time
import random
import numpy as np
import numpy.random as npr
import argparse
import cv2

import torch

import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import re
from graphviz import Digraph, Source
from shapely.geometry import box

# Root directory of the project
import os
import sys
ROOT_DIR = os.path.abspath(os.path.join(os.path.realpath(__file__), '../..')) 
LIB_PATH = os.path.join(ROOT_DIR, "msdn")
print (LIB_PATH)
sys.path.append(LIB_PATH)

dir(sys.modules[__name__])

from faster_rcnn import network
from faster_rcnn.MSDN import Hierarchical_Descriptive_Model
from faster_rcnn.utils.timer import Timer
from faster_rcnn.fast_rcnn.config import cfg
from faster_rcnn.datasets.visual_genome_loader import visual_genome
from faster_rcnn.utils.HDN_utils import get_model_name, group_features
from visualize_cv import draw_bbox_label_msdn
import std_msgs.msg
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_safety.msg import VelocityScale

os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"]="0"

TIME_IT = cfg.TIME_IT
parser = argparse.ArgumentParser('Options for training Hierarchical Descriptive Model in pytorch')


#Training parameters

parser.add_argument('--lr', type=float, default=0.001, metavar='LR', help='base learning rate for training')
parser.add_argument('--max_epoch', type=int, default=10, metavar='N', help='max iterations for training')
parser.add_argument('--momentum', type=float, default=0.99, metavar='M', help='percentage of past parameters to store')
parser.add_argument('--log_interval', type=int, default=1000, help='Interval for Logging')
parser.add_argument('--step_size', type=int, default = 1, help='Step size for reduce learning rate')
parser.add_argument('--resume_training', action='store_true', default = True, help='Resume training from the model [resume_model]')
parser.add_argument('--resume_model', type=str, default=os.path.join(ROOT_DIR, "models/100_epoch_best.h5"), help='The model we resume')
parser.add_argument('--load_RPN', action='store_true', help='To end-to-end train from the scratch')
parser.add_argument('--enable_clip_gradient', action='store_true', help='Whether to clip the gradient')
parser.add_argument('--use_normal_anchors', action='store_true', help='Whether to use kmeans anchors')

# structure settings
parser.add_argument('--disable_language_model', action='store_true', help='To disable the Lanuage Model ')
parser.add_argument('--mps_feature_len', type=int, default=1024, help='The expected feature length of message passing')
parser.add_argument('--dropout', action='store_true', help='To enables the dropout')
parser.add_argument('--MPS_iter', type=int, default=1, help='Iterations for Message Passing')
parser.add_argument('--gate_width', type=int, default=128, help='The number filters for gate functions in GRU')
parser.add_argument('--nhidden_caption', type=int, default=512, help='The size of hidden feature in language model')
parser.add_argument('--nembedding', type=int, default=256, help='The size of word embedding')
parser.add_argument('--rnn_type', type=str, default='LSTM_normal', help='Select the architecture of RNN in caption model[LSTM_im | LSTM_normal]')
parser.add_argument('--caption_use_bias', action='store_true', default=True, help='Use the flap to enable the bias term to caption model')
parser.add_argument('--caption_use_dropout', action='store_const', const=0.5, default=0.5, help='Set to use dropout in caption model')
parser.add_argument('--enable_bbox_reg', dest='region_bbox_reg', action='store_true')
parser.add_argument('--disable_bbox_reg', dest='region_bbox_reg', action='store_false')
parser.set_defaults(region_bbox_reg=True)
parser.add_argument('--use_kernel_function', action='store_true')
# Environment Settings
parser.add_argument('--seed', type=int, default=1, help='set seed to some constant value to reproduce experiments')
parser.add_argument('--saved_model_path', type=str, default = './models/RPN/RPN_region_best.h5', help='The Model used for initialize')
parser.add_argument('--dataset_option', type=str, default='normal', help='The dataset to use (small | normal | fat)')
parser.add_argument('--output_dir', type=str, default='./models/HDN_resume', help='Location to output the model')
parser.add_argument('--model_name', type=str, default='HDN', help='The name for saving model.')
parser.add_argument('--nesterov', action='store_true', help='Set to use the nesterov for SGD')
parser.add_argument('--finetune_language_model', action='store_true', help='Set to disable the update of other parameters')
parser.add_argument('--optimizer', type=int, default=0, help='which optimizer used for optimize language model [0: SGD | 1: Adam | 2: Adagrad]')



class ros_msdn:

    def __init__(self):
        # To set the model name automatically
        args = parser.parse_args()
        print args
        args = get_model_name(args)
        print 'Model name: {}'.format(args.model_name)
        self.check = True

        # To set the random seed
        random.seed(args.seed)
        torch.manual_seed(args.seed + 1)
        torch.cuda.manual_seed(args.seed + 2)

        print("Loading training params"),
        self.train_set = visual_genome('normal', 'train')
        print("Done.")

        self.train_loader = torch.utils.data.DataLoader(self.train_set, batch_size=1, shuffle=True, num_workers=8, pin_memory=True)
        end = time.time()
        # Model declaration
        self.net = Hierarchical_Descriptive_Model(nhidden=args.mps_feature_len,
                     n_object_cats=self.train_set.num_object_classes, 
                     n_predicate_cats=self.train_set.num_predicate_classes,
                     n_vocab=self.train_set.voc_size,
                     voc_sign=self.train_set.voc_sign,
                     max_word_length=self.train_set.max_size, 
                     MPS_iter=args.MPS_iter, 
                     use_language_loss=not args.disable_language_model,
                     object_loss_weight=self.train_set.inverse_weight_object, 
                     predicate_loss_weight=self.train_set.inverse_weight_predicate,
                     dropout=args.dropout,
                     use_kmeans_anchors=not args.use_normal_anchors,
                     gate_width = args.gate_width,
                     nhidden_caption = args.nhidden_caption,
                     nembedding = args.nembedding,
                     rnn_type=args.rnn_type, 
                     rnn_droptout=args.caption_use_dropout, rnn_bias=args.caption_use_bias, 
                     use_region_reg = args.region_bbox_reg, 
                     use_kernel = args.use_kernel_function)

        params = list(self.net.parameters())
        for param in params:
            print param.size()
        print self.net

        # To group up the features
        vgg_features_fix, vgg_features_var, rpn_features, hdn_features, language_features = group_features(self.net)

        # Setting the state of the training model
        self.net.cuda()
        self.net.train()
        network.set_trainable(self.net, False)

        # loading model for inference
        print 'Resume training from: {}'.format(args.resume_model)
        if len(args.resume_model) == 0:
            raise Exception('[resume_model] not specified')
        network.load_net(args.resume_model, self.net)
        args.train_all = True
        optimizer_select = 2

        optimizer = network.get_optimizer(args.lr,optimizer_select, args, 
                    vgg_features_var, rpn_features, hdn_features, language_features)

        target_net = self.net
        self.net.eval()
        print ('Model Loading time: ', time.time() - end)

        # Set topics
        self.bridge = CvBridge()
        self.dot = Digraph(comment='warehouse', format='svg')
        self.regions_dot = Digraph(comment='regions', format='svg')

        self.image_sub = message_filters.Subscriber('/turtlebot2i/camera/rgb/raw_image', Image)
        self.image_depth_sub = message_filters.Subscriber('/turtlebot2i/camera/depth/raw_image', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.image_depth_sub], queue_size=1)
        print ('calling callback')
        self.ts.registerCallback(self.callback)
        self.scenegraph_pub = rospy.Publisher('/turtlebot2i/scene_graph', SceneGraph, queue_size=10)


    def callback(self, image, depth_image):

        try:
            print 'inside callback '
            farClippingPlane = 3.5
            nearClippingPlane = 0.0099999
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,"passthrough")
            cv_depth_image = cv2.flip(cv_depth_image, 0)
            cv_depth_image = nearClippingPlane  + (cv_depth_image * (farClippingPlane - nearClippingPlane))
            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")

            predicates_frequency = {'behind' : 1, 'on': 1, 'has': 1000000, 'in_front_of' : 1, 'next_to': 2, 'beside': 2, 'with': 1, 'attach_to': 1, 'connected_to': 1, 'charges': 1, 'in_hands_of': 1}
            all_classes = {'slidingdoor':0, 'wall':0, 'shelf':0, 'robot':0, 'human':0, 'conveyorbelt':0, 'dockstation':0, 'product':0, 'floor':0}
            class_names = ['floor', 'wall', 'shelf', 'robot', 'human', 'conveyorbelt', 'dockstation', 'product', 'slidingdoor' ]
            allowed_self_relationship = {'slidingdoor':[], 'wall':['beside','attach_to'], 'shelf':['beside','next_to'], 'robot':[], 'human':['in_front_of','behind'], 'conveyorbelt':[], 'dockstation':[], 'product':['beside','next_to'], 'floor':[]}

            print("Describing.....")
            if self.check == False:
                self.dot.clear()
                self.regions_dot.clear()
            im, im_info = self.train_set.get_image_info(cv_image)

            end = time.time()

            region_caption, region_list, region_pred_boxes, region_logprobs, class_pred_boxes, class_scores,\
             class_inds, subject_list, object_list, predicate_list, predicate_inds, predicate_scores = self.net.describe(im.unsqueeze(0), [im_info], top_N=[50])


            class_idx = []
            for class_ in all_classes.keys():
                class_idx.append(self.train_set.word2idx[class_])

            predicate_idx = []
            for predicate in predicates_frequency.keys():
                predicate_idx.append(self.train_set.word2idx[predicate])
            
            classes_name = []

            predicate_scores = predicate_scores.squeeze()[predicate_list]
            subject_scores = class_scores[subject_list].squeeze()
            object_scores = class_scores[object_list].squeeze()
            relationship_scores = predicate_scores * (subject_scores + object_scores)/2.0

            keep_indexes = np.where((subject_scores > 0.7) & (object_scores > 0.7) &  (predicate_scores > 0.5))[0]
            keep_classes = np.where(class_scores > 0.7)[0]
            class_name_score = dict()
            for i in keep_classes:
                class_name = self.train_set._object_classes[class_inds[i]]
                score = class_scores[i]
                all_classes[class_name] += 1
                if class_name != 'floor':
                    classes_name.append(str(class_name +'#'+ str(all_classes[class_name])))
                    class_name_score[str(class_name +'#'+ str(all_classes[class_name]))] = score
                else:
                    classes_name.append(str(class_name))
                    class_name_score[str(class_name)] = score

            #_ = draw_bbox_label_msdn(cv_image, class_pred_boxes[keep_classes], class_inds[keep_classes], class_scores[keep_classes])

            classes_name = np.array(classes_name)
            subject_scores = subject_scores[keep_indexes]
            object_scores = object_scores[keep_indexes]
            subject_list = subject_list[keep_indexes]
            object_list = object_list[keep_indexes]
            predicate_list = predicate_list[keep_indexes]
            relationship_scores = relationship_scores[keep_indexes]
            predicate_scores = predicate_scores[keep_indexes]

            # subject_inds = class_inds[subject_list]
            # object_inds = class_inds[object_list]

            subjects_name = classes_name[subject_list]
            objects_name = classes_name[object_list]
            predicate_inds = predicate_inds.squeeze()[predicate_list]

            #print (class_inds[subject_list[keep_indexes]])
            relationship_dict = dict()

            last_subject = ''
            last_predicate = ''
            temp_score_list = []
            object_ids = []

            for i in range (len(subjects_name)):

                predicate = self.train_set._predicate_classes[predicate_inds[i]]
                subject = subjects_name[i]
                _object = objects_name[i]

                if subject != _object:
                    if (subject == 'floor' and predicate != 'has') or (_object == 'floor' and predicate != 'on') or (subject[:-2] == 'wall' and predicate == 'in_front_of' and _object[:-2] == 'dockstation'):
                        print 'unwanted relationship', subject, '-> ' , predicate, ' -> ', _object

                    elif subject == 'floor' and predicate == 'has':
                        if subject not in relationship_dict.keys():
                            relationship_dict[subject] = dict()
                            relationship_dict[subject][predicate] = dict()
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]

                        elif predicate not in relationship_dict[subject].keys():
                            relationship_dict[subject][predicate] = dict()
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]

                        elif _object not in relationship_dict[subject][predicate].keys():
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]

                        else:
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]

                    elif _object in relationship_dict.keys() and predicate in relationship_dict[_object].keys()\
                             and subject in relationship_dict[_object][predicate].keys() and  predicate_scores[i] > relationship_dict[_object][predicate][subject]:
                        
                        if subject not in relationship_dict.keys():
                            relationship_dict[subject] = dict()
                            relationship_dict[subject][predicate] = dict()
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]
                        elif predicate not in relationship_dict[subject].keys():
                            relationship_dict[subject][predicate] = dict()
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]

                        elif _object not in relationship_dict[subject][predicate].keys():
                            relationship_dict[subject][predicate][_object] = predicate_scores[i]
                        
                        del relationship_dict[_object][predicate][subject]
                        if len(relationship_dict[_object][predicate]) == 0:
                            del relationship_dict[_object][predicate]
                            if len(relationship_dict[_object]) == 0:
                                del relationship_dict[_object]

                    elif subject == last_subject:
                        if predicate == last_predicate:
                            temp_score_list.append(predicate_scores[i])
                            object_ids.append(i)
                        else:
                            if len(temp_score_list) > 1:
                                sorted_scores = np.array(temp_score_list).argsort()[::-1]
                                indx = np.array(object_ids)[sorted_scores][0]
                            else:
                                indx = object_ids[-1]
                            #print 'Saving relationship 1', subject, '-> ' , last_predicate, ' -> ', objects_name[indx]

                            relationship_dict[subject][last_predicate][objects_name[indx]] = predicate_scores[indx]

                            relationship_dict[subject][predicate] = dict()
                            temp_score_list = [predicate_scores[i]]

                            if subject[:-2] == _object[:-2] and predicate not in allowed_self_relationship[subject[:-2]]:
                                object_ids = []
                                last_predicate = ''
                                last_subject = ''
                            else:
                                object_ids = [i]
                                last_predicate = predicate
                    else:
                        relationship_dict[subject] = dict()
                        relationship_dict[subject][predicate] = dict()
                        if last_subject != '':
                            if len(temp_score_list) > 1:
                                sorted_scores = np.array(temp_score_list).argsort()[::-1]
                                indx = np.array(object_ids)[sorted_scores][0]
                            else:
                                indx = object_ids[-1]

                            #print 'Saving relationship 2', last_subject, '-> ' , last_predicate, ' -> ', objects_name[indx]

                            relationship_dict[last_subject][last_predicate][objects_name[indx]] = predicate_scores[indx]

                        if subject[:-2] == _object[:-2] and predicate not in allowed_self_relationship[subject[:-2]]:
                            object_ids = []
                            last_predicate = ''
                            last_subject = ''
                            temp_score_list = []
                        else:
                            last_subject = subject
                            last_predicate = predicate
                            temp_score_list = [predicate_scores[i]]
                            object_ids = [i]

            if last_subject != '':
                if len(temp_score_list) > 1:
                    sorted_scores = np.array(temp_score_list).argsort()[::-1]
                    indx = np.array(object_ids)[sorted_scores][0]
                else:
                    indx = object_ids[-1]
                #print 'Saving relationship 3', last_subject, '-> ' , last_predicate, ' -> ', objects_name[indx]

                relationship_dict[last_subject][last_predicate][objects_name[indx]] = predicate_scores[indx]

            print ('Time taken to decribe: ', time.time() - end)


            self.dot.node_attr['shape']='record'
            robot_label = "turtlebot2i"
            
            #self.dot.node('robot', label=robot_label)
            self.dot.node('warehouse', label='warehouse')
            floor_label = "{floor|Score: 0.7}"
            if 'floor' in class_name_score.keys():
                floor_label = '%s|Score: %.2f'%( 'floor',  class_name_score['floor'])

            self.dot.node('floor', label=floor_label)
            self.dot.edge('warehouse','floor')

            list_nodes = ['warehouse', 'floor']

            for subject in relationship_dict.keys():
                for predicate in relationship_dict[subject].keys():
                    for _object in relationship_dict[subject][predicate].keys():
                        if subject not in list_nodes:
                            node_label = '%s|Score: %.2f'%( subject,  class_name_score[subject])
                            self.dot.node(subject, label=node_label)
                            list_nodes.append(subject)
                        if _object not in list_nodes:
                            node_label = '%s|Score: %.2f'%( _object,  class_name_score[_object])
                            self.dot.node(_object, label=node_label)
                            list_nodes.append(_object)
                        self.dot.edge(subject, _object, label=predicate)

                        print 'Subject : ', subject, ' Predicate: ', predicate,' Object: ', _object, ' Score: ',relationship_dict[subject][predicate][_object]
            print 'END PRINTING Relationships...'

            
            sorted_regions = region_logprobs.argsort()[::-1]
            regions_dict = dict()
            regions_prob_dict = dict()
            sorted_region_keys = []

            for i in sorted_regions:
                if region_logprobs[i] > -0.5:
                    region_idx = region_caption[i]
                    common = list( frozenset(region_idx) & frozenset(class_idx))
                    #print 'Common classes: ', common
                    if len(common) == 2: 
                        class_1 = self.train_set.idx2word[common[0]]
                        class_2 = self.train_set.idx2word[common[1]]
                        if all_classes[class_1] != 0 and all_classes[class_2] != 0:

                            key = frozenset([class_1, class_2])
                            #print key
                            if key not in regions_prob_dict.keys():
                                regions_prob_dict[key] = region_logprobs[i]
                                regions_dict[key] = region_caption[i]
                                sorted_region_keys.append(key)

                            elif regions_prob_dict[key] < region_logprobs[i]:
                                regions_prob_dict[key] = region_logprobs[i]
                                regions_dict[key] = region_caption[i]
                                sorted_region_keys.append(key)

                    elif len(common) == 1:
                        class_1 = self.train_set.idx2word[common[0]]
                        if all_classes[class_1] != 0:
                            key = frozenset([class_1])
                            #print key
                            if key not in regions_prob_dict.keys():
                                regions_prob_dict[key] = region_logprobs[i]
                                regions_dict[key] = region_caption[i]
                                sorted_region_keys.append(key)

                            elif all_classes[class_1] > 1:
                                j = 1
                                while j < all_classes[class_1]:
                                    key = frozenset([class_1+'#'+str(j)])
                                    #print key

                                    if key not in regions_prob_dict.keys():
                                        regions_prob_dict[key] = region_logprobs[i]
                                        regions_dict[key] = region_caption[i]
                                        sorted_region_keys.append(key)

                                    elif regions_prob_dict[key] < region_logprobs[i]:
                                        regions_prob_dict[key] = region_logprobs[i]
                                        regions_dict[key] = region_caption[i]
                                        sorted_region_keys.append(key)

                                    j+=1

            self.regions_dot.node_attr['shape']='record'
            captions_list = []

            for key in sorted_region_keys:
                region_idx = regions_dict[key]
                log_prob = regions_prob_dict[key]
                caption = ""
                space = ""
                for indx in region_idx:
                    word = self.train_set.idx2word[indx]
                    if word != "<unknown>" and word != "<start>" and word != "<end>":
                        caption += space + word
                        space = " "
                node_label = "%s|Log probability: %.6f"%(caption,  log_prob)
                repetition_check = True
                if caption not in captions_list:
                    self.regions_dot.node(caption, label=node_label)
                    repetition_check = False

                if len(captions_list) > 0:
                    self.regions_dot.edge(captions_list[-1], caption)

                if repetition_check == False:
                    captions_list.append(caption)

                #print caption, log_prob
            self.dot.render('scene_graph.gv', view = self.check)
            self.regions_dot.render('region_graph.gv', view = self.check)

            #s = Source(self.dot, filename="scene_graph", format="png")
            #s1 = Source(self.regions_dot, filename="region_graph", format="png")
            # #if self.check == False:
            # s.view()
            # s1.view()

            if self.check == True:
                self.check = False
            print 'END PRINTING Regions...'

        except CvBridgeError as e:
            print(e)
if __name__ == '__main__':
    print 'start'
    rospy.init_node('msdn_py')
    detector = ros_msdn()
    rospy.spin()
