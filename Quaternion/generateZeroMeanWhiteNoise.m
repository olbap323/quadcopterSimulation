
accVar = 400*g/1e6;
gyroVar = .033 * pi/180;
whiteNoise = zeros(6,No);

whiteNoise(1:3,:) = randn(3,No)*accVar;
whiteNoise(4:6,:) = randn(3,No)*gyroVar;


% figure();
% for i = 1:6,
%    subplot(3,2,i);
%    plot(whiteNoise(i,:))
% end
