filePath1 = uigetdir;
fileExt = '*.jpg'; 
files = dir(fullfile(filePath1,fileExt));
len1 = size(files,1);
for i=1:len1
   fileName = strcat(filePath1,files(1).name); %strcat���ַ�������
   image = imread(fileName);%��ȡͼ������
   imshow(image);%һ������ʾͼƬ
end
