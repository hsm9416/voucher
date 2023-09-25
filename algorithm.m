% 시간 벡터 생성 (예: 0초에서 20초까지 0.1초 간격으로)
t = 0:0.1:20;

% 2초마다 0에서 100으로 선형적으로 증가하는 값 생성
y = 2 * 0.5556 * mod(t, 1);

% 그래프 그리기
figure;
plot(t, y);
xlabel('Time (seconds)');
ylabel('Value');
title('Linear increase with 2 second period');
grid on;

