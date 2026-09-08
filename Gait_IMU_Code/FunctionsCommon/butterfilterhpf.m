function y=butterfilterhpf(data,cutoff,fs,order)
if nargin~=4
    error('4 inputs required!')
end
[b,a]=butter(order,cutoff/(fs/2),'high');
y=filtfilt(b,a,data);