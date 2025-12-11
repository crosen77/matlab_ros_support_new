function [all_bboxes, all_scores, all_labels, numObjects, myImg, annotatedImage] = getLabeledImg(zone, optns)
% ------------------------------------------------------------------------
% Takes a picture by accessing a subscriber, loads the dector and labels
% the image taken, and displays it.
% 
% Input: 
%   optns - to access the subscriber to take a pic through the
%                rosClass
% Outputs:
%   bboxes - P-by-4 matrix defining P bounding boxes. Each row of bboxes
%              contains a four-element vector, [x, y, width, height]. This
%              vector specifies the upper-left corner and size of a bounding
%              box in pixels 
%   scores - confidence scores for each bounding box (PX1)
%   labels - labels assigned to the bounding boxes (PX1)
%   numObjects - number of objects detected ( would also be P )
%   myImg - image taken without any labels
%   annotatedImage - myImg but with bounding boxes, scores, and labels for
%               each objects detected
% ------------------------------------------------------------------------
    %% Get the ROS Class handel
    r = optns{'rHandle'};

    %% Take picture and Read the Image
    pause(2);
    disp("Taking picture...")

    rosImg  = receive(r.rgb_sub);
    myImg   = rosReadImage(rosImg,"PreserveStructureOnRead",true);
    
    %% Bounding Boxes
    disp("Computing bounding boxes, scores, and labels...")
    
    %% According to strategy leverage different detectors...
    pretrained_detectors = r.detectors;
    all_bboxes = [];
    all_scores = [];
    all_labels = categorical([]);

    for detector = 1:size(pretrained_detectors, 1)

        trainedYoloNet = pretrained_detectors{detector}.detector;
        %% TODO: Detect objects using yolo. Output bboxes, scores, labels. Threshold of 0.7
        if strcmp(zone, 'Zone3Pouch') & detector == 1
            [bboxes,scores,labels] = detect(trainedYoloNet, myImg, Threshold=0.03);
        elseif strcmp(zone, 'Zone4')
            [bboxes,scores,labels] = detect(trainedYoloNet, myImg, Threshold=0.6);
        else
            [bboxes,scores,labels] = detect(trainedYoloNet, myImg, Threshold=0.5);
        end


        %% Add the information to the final arrays
        bbox_len = size(bboxes,1);
        all_bbox_len = size(all_bboxes,1);
        all_bboxes(all_bbox_len + 1 : all_bbox_len + bbox_len, :) = bboxes;

        score_len = size(scores, 1);
        all_score_len = size(all_scores, 1);
        all_scores(all_score_len + 1 : all_score_len + score_len, :) = scores;

        label_len = size(labels, 1);
        all_label_len = size(all_labels, 1);
        all_labels(all_label_len + 1 : all_label_len + label_len, :) = labels;

    end
    
    %% TODO: Visualize the detected objects' bounding boxes by calling insertObjectAnnotation and save to annotatedImage
    disp("Drawing bounding boxes...")
    annotatedImage = insertObjectAnnotation(im2uint8(myImg), ...
                                            "rectangle", all_bboxes, ...
                                             string(all_labels) + ":" + string(all_scores));

    %% Display
    if optns{'debug'}
        figure(1), imshow(annotatedImage);
    end


    %% Specify percentage of acceptable bounding box
    numObjects = size(all_bboxes,1);
end



