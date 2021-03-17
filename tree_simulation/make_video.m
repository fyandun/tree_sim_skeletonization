%%
clear all
bag_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/lab_vine/bagFiles/';

file = 'x_3_1_transcient';
filename_init = strcat(bag_path, file, '.bag');
bag_init_conds = rosbag(filename_init);
bSel_init = select(bag_init_conds,'Topic','/theia/cam0/image_rect');
msgStructs = readMessages(bSel_init);
%%
videoOutPath = '/home/fyandun/Documentos/Manipulator/papers/cvpr2020/presentation/images/';

video_file = strcat(videoOutPath,file, '.avi');
v = VideoWriter(video_file);
v.FrameRate = 10;
open(v)

end_iter = length(msgStructs);
for i=1:end_iter
    tmp_ = msgStructs{i}.Data;
    tmp_ = reshape(tmp_, msgStructs{i}.Width, msgStructs{i}.Height);
    tmp = tmp_(200:end,200:end);
    img_ = imrotate(tmp, -90);
    img = adapthisteq(img_);
    img = flipdim(img,2);
    %imshow(adapthisteq(img))
    writeVideo(v,img);
end
close(v)