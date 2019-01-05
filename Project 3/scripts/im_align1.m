function [offsetg_along_x,offsetg_along_y,offsetr_along_x,offsetr_along_y,ssdimage] = im_align1(r,g,b)
  
  # Finding the size of the R G B channels
  [px,py] = size(b);
  
  offset = 30;
  
  # Getting rid of the borders in the channel
  newb = b(offset:px-offset,offset:py-offset);
  [offx,offy] = size(newb);
  
  # Setting the SSD to the maximum possible value
  ssdg = 255 * offx * offy;
  ssdr = 255 * offx * offy;
  
  for i = -15:15
    for j = -15:15
      newg = g(offset+i:px-offset+i , offset+j:py-offset+j);
      newr = r(offset+i:px-offset+i , offset+j:py-offset+j);
      
      ## Reference from https://www.mathworks.com/help/matlab/ref/nextpow2.html
      offg = (newb - newg).^2;
      offr = (newb - newr).^2;
      
      temp_ssdg = sum(sum(offg));
      temp_ssdr = sum(sum(offr));
      
      if temp_ssdg <= ssdg
        ssdg = temp_ssdg;
        offsetg_along_x = i;
        offsetg_along_y = j;
      endif
      
      if temp_ssdr <= ssdr
        ssdr = temp_ssdr;
        offsetr_along_x = i;
        offsetr_along_y = j;
      endif
      
    endfor
  endfor
  
  # Finding the new R and G channel offsets using the offset calculated by SSD
  newg = g(offset+offsetg_along_x:px-offset+offsetg_along_x , offset+offsetg_along_y:py-offset+offsetg_along_y);
  newr = r(offset+offsetr_along_x:px-offset+offsetr_along_x , offset+offsetr_along_y:py-offset+offsetr_along_y);
  
  ssdimage = cat(3,newr,newg,newb);
endfunction