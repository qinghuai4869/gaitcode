function y=butterfilterlpf(data,cutoff,fs,order)

if nargin~=4
    error('4 inputs required!')
end
[b,a]=butter(order,cutoff/(fs/2));
y=filtfilt(b,a,data);