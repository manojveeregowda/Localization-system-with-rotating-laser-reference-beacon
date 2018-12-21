n=0:5:359;
lut=zeros(numel(n),5);

x = 1;
for dx = n
    lut(x,1) = dx;
    lut(x,2:5) = 0;
    
    a = floor(1023 * [cosd(dx), sind(dx)]);
    a1 = a(1);
    a2 = a(2);
    
    if a1 >= 0
        lut(x,2) = a1;
    else
        lut(x,3) = abs(a1);
    end
    
    if a2 >= 0
        lut(x,4) = a2;
    else
        lut(x,5) = abs(a2);
    end
    x = x + 1;
end

for x = 1:numel(lut(:,1))
    fprintf('\tsteps[%d] = ( struct TableStep ){ .a1 = %d, .a2 = %d, .b1 = %d, .b2 = %d };\n',...
        x - 1, lut(x,2), lut(x,3), lut(x,4), lut(x,5))
end