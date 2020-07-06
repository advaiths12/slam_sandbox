function y = back_sub(A, b)

n = length(b);
y = zeros(n,1);
y(n) = b(n)/A(n,n);

for i = n-1:-1:1
    y(i) = (b(i)-A(i,i+1:n)*y(i+1:n))./A(i,i);
end