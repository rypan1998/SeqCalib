#!/bin/bash
DATASET_PATH=../data
CODE_PATH=..
groupNum=10
groupStart=0
cameraNum=60
cameraStart=0
img_idx=0

# 步骤一
echo "----- step1: construct colmap database +++++"
mkdir ./$PROJECT_NAME
cp -r $DATASET_PATH/$img_idx ./$PROJECT_NAME/color
echo ${DATASET_PATH}
colmap feature_extractor \
    --database_path $DATASET_PATH/$PROJECT_NAME/database.db \
    --image_path $DATASET_PATH/$img_idx\
    --ImageReader.camera_model SIMPLE_PINHOLE

colmap exhaustive_matcher \
    --database_path $DATASET_PATH/$PROJECT_NAME/database.db

#步骤2
echo "----- step2: generate random 3D points to database +++++"
${CODE_PATH}/build/Extract \
    --cam_num $cameraNum \
    --cam_start $cameraStart \
    --group_num $groupNum \
    --group_start $groupStart \
    --image_path $DATASET_PATH/%d/%04d.jpg \
    --project_path $DATASET_PATH/$PROJECT_NAME \
    --is_aruco 1 \
    --max_points 21000 \
    --pixel_error 1 \
    --track_length 7 20 \
    --axis_range -500 500 -500 500 0 1000

# 步骤3
echo "----- step3: Construction +++++"
DATASET_PATH="${DATASET_PATH}/$PROJECT_NAME"

echo "----- step3.1: matches_importer +++++"
colmap matches_importer --database_path $DATASET_PATH/database.db \
                        --match_list_path $DATASET_PATH/match.txt \
                        --match_type 'inliers'

# colmap automatic_reconstructor --workspace_path $DATASET_PATH \
#                                 --image_path $DATASET_PATH/color \
#                                 --quality extreme \
#                                 --dense 0 \
#                                 --camera_model OPENCV
echo "----- step 3.2: mapper +++++"
colmap mapper --database_path $DATASET_PATH/database.db \
                --image_path $DATASET_PATH/color\
                --output_path $DATASET_PATH \
                --Mapper.ba_local_max_refinements 3 \
                --Mapper.ba_local_max_num_iterations 100 \
                --Mapper.max_extra_param 99999 \
                --Mapper.ba_refine_principal_point false \
                --Mapper.num_threads -1

# mkdir $DATASET_PATH/txt
echo "----- step 3.3: model_converter +++++"
colmap model_converter --input_path "${DATASET_PATH}/0" \
                        --output_path "$DATASET_PATH/0" \
                        --output_type TXT
