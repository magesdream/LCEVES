filePath1 = uigetdir;
fileExt = '*.jpg'; 
files = dir(fullfile(filePath1,fileExt));
len1 = size(files,1);
for i=1:len1
   fileName = strcat(filePath1,files(1).name); %strcat：字符串附加
   image = imread(fileName);%读取图像数据
   imshow(image);%一张张显示图片
end
