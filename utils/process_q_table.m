folder = "data";
username = "shen";

avg_q_value_mode1 = zeros(1, 54);

for i = 1:54
    filename = sprintf('%s/mode1/%s-q_table_value-%d.txt', folder, username, i);
    tmp = importdata(filename);
    q_table_data = tmp.data;
    avg_value = sum(sum(q_table_data))/100;
    avg_q_value_mode1(i) = avg_value;
end
plot(avg_q_value_mode1);
hold on;

avg_q_value_mode2 = zeros(1, 54);

for i = 1:54
    filename = sprintf('%s/mode2/%s-q_table_value-%d.txt', folder, username, i);
    tmp = importdata(filename);
    q_table_data = tmp.data;
    avg_value = sum(sum(q_table_data))/100;
    avg_q_value_mode2(i) = avg_value;
end
plot(avg_q_value_mode2);
hold on;

avg_q_value_mode3 = zeros(1, 51);

for i = 1:51
    filename = sprintf('%s/mode3/%s-q_table_value-%d.txt', folder, username, i);
    tmp = importdata(filename);
    q_table_data = tmp.data;
    avg_value = sum(sum(q_table_data))/100;
    avg_q_value_mode3(i) = avg_value;
end
plot(avg_q_value_mode3);
hold on;

avg_q_value_mode4 = zeros(1, 53);

for i = 1:53
    filename = sprintf('%s/mode4/%s-q_table_value-%d.txt', folder, username, i);
    tmp = importdata(filename);
    q_table_data = tmp.data;
    avg_value = sum(sum(q_table_data))/100;
    avg_q_value_mode4(i) = avg_value;
end
plot(avg_q_value_mode4);
hold on;

avg_q_value_mode_test = zeros(1, 53);

for i = 1:199
    filename = sprintf('%s/mode-test/test-q_table_value-%d.txt', folder, i);
    tmp = importdata(filename);
    q_table_data = tmp.data;
    avg_value = sum(sum(q_table_data))/100;
    avg_q_value_mode_test(i) = avg_value;
end
plot(avg_q_value_mode_test);
hold on;