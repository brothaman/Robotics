% 100 equi-spaced increments from 0 to pi

a = zeros(100);

for i = 0:100
    a(i+1) = (pi*i/1000.0);
end

for theta = 1:length(a)
    my_rotation(theta)
end
