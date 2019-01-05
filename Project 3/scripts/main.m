clc
close all
clear all

pkg load image

for images = 1:6
  img = imread(strcat('image',int2str(images),'.jpg'));
  [x,y] = size(img);
  
  # Extracting R G B planes from the image
  b = img(1:floor(x/3)+1 , 1:y);
  g = img(floor(x/3):2*floor(x/3) , 1:y);
  r = img(2*floor(x/3):x-1 , 1:y);
  
  # Merging the R G B planes into one image
  colorimg = cat(3,r,g,b);
  imwrite(colorimg, strcat('image',int2str(images),'-color.jpg'));
  
  # Performing SSD alignment on the image
  [offsetg_along_x,offsetg_along_y,offsetr_along_x,offsetr_along_y,ssdimage] = im_align1(r,g,b);
  disp('');
  disp(strcat('Image',int2str(images)));
  disp(strcat('G channel alignment shift using SSD: [',int2str(offsetg_along_x),',',int2str(offsetg_along_y),']'));
  disp(strcat('R channel alignment shift using SSD: [',int2str(offsetr_along_x),',',int2str(offsetr_along_y),']'));
  imwrite(ssdimage, strcat('image',int2str(images),'-ssd.jpg'));
  
  # Performing NCC alignment on the image
  [offsetg_along_x,offsetg_along_y,offsetr_along_x,offsetr_along_y,normedimage] = im_align2(r,g,b);
  disp('');
  disp(strcat('G channel alignment shift using NCC: [',int2str(offsetg_along_x),',',int2str(offsetg_along_y),']'));
  disp(strcat('R channel alignment shift using NCC: [',int2str(offsetr_along_x),',',int2str(offsetr_along_y),']'));
  disp('');
  imwrite(normedimage, strcat('image',int2str(images),'-ncc.jpg'));
endfor