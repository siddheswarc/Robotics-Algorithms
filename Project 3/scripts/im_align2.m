function [offsetg_along_x,offsetg_along_y,offsetr_along_x,offsetr_along_y,normedimage] = im_align2(r,g,b)
  
  # Finding the size of the R G B channels
  [px,py] = size(b);
  
  offset = 30;
  
  # Getting rid of the borders in the channel
  newb = b(offset:px-offset,offset:py-offset);
  
  # Making templates of the R and G channels
  templateg = g(100:150,100:150);
  templater = r(100:150,100:150);
  
  # Calculating the Normalized Cross Correlation of R and G channels with B channel
  ## Reference from https://www.mathworks.com/help/images/ref/normxcorr2.html
  cg = normxcorr2(templateg, b);
  [gxpeak, gypeak] = find(cg==max(cg(:)));
  
  cr = normxcorr2(templater, b);
  [rxpeak, rypeak] = find(cr==max(cr(:)));
  
  # Calculating the offset of R and G channels with respect to the template size
  ## Reference from https://www.mathworks.com/help/images/ref/normxcorr2.html
  ## offset = starting location of template minus 'peak' calculated using normxcorr2() plus width/height of template
  offsetg_along_x = 100-gxpeak+50;
  offsetg_along_y = 100-gypeak+50;
  
  offsetr_along_x = 100-rxpeak+50;
  offsetr_along_y = 100-rypeak+50;
  
  # Finding the new R and G channel offsets using the offset calculated by NCC
  normedg = g(offset+offsetg_along_x:px-offset+offsetg_along_x , offset+offsetg_along_y:py-offset+offsetg_along_y);
  normedr = r(offset+offsetr_along_x:px-offset+offsetr_along_x , offset+offsetr_along_y:py-offset+offsetr_along_y);
  
  normedimage = cat(3,normedr,normedg,newb);
endfunction