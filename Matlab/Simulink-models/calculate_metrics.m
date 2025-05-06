function calculate_metrics(X, Xd, U, t, Q, R)
    % Расчет ошибки
    error = X - Xd;
    
    % 1. Максимальное перерегулирование (в %)
    overshoot = zeros(1,4);
    for i = 1:4
        if all(Xd(:,i) == Xd(1,i)) % Если эталон постоянный
            max_val = max(abs(X(:,i) - Xd(1,i)));
            final_val = abs(X(end,i) - Xd(1,i));
            if max_val > final_val
                overshoot(i) = ((max_val - final_val) / final_val) * 100;
            end
        else % Для переменного эталона
            rel_error = abs(error(:,i)) ./ (abs(Xd(:,i)) + eps);
            overshoot(i) = max(rel_error) * 100;
        end
    end
    fprintf('Максимальное перерегулирование:\n');
    fprintf('  x: %.2f%%, y: %.2f%%, z: %.2f%%, psi: %.2f%%\n', overshoot(1), overshoot(2), overshoot(3), overshoot(4));
    
    % 2. Время переходного процесса (когда ошибка окончательно вошла в 5% коридор)
    settling_threshold = 0.05;
    settling_time = cell(1,4);
    for i = 1:4
        if all(Xd(:,i) == Xd(1,i)) % Для постоянного эталона
            target = Xd(1,i);
            error_abs = abs(X(:,i) - target);
            threshold = settling_threshold * abs(target);
        else % Для переменного эталона
            error_abs = abs(error(:,i));
            threshold = settling_threshold * abs(Xd(:,i));
        end
        
        % Находим все моменты когда ошибка входит в коридор
        in_band = error_abs <= threshold;
        
        % Ищем последний переход из вне коридора в коридор
        transitions = diff([false; in_band]);
        last_entry = find(transitions == 1, 1, 'last');
        
        if ~isempty(last_entry) && all(in_band(last_entry:end))
            settling_time{i} = sprintf('%.2f с', t(last_entry));
        else
            settling_time{i} = 'never';
        end
    end
    fprintf('\nВремя переходного процесса (5%%):\n');
    fprintf('  x: %s, y: %s, z: %s, psi: %s\n', ...
            settling_time{1}, settling_time{2}, settling_time{3}, settling_time{4});
    
    % 3. Характер процесса (монотонный, колебательный)
    fprintf('\nХарактер процесса:\n');
    for i = 1:4
        error_diff = diff(error(:,i));
        sign_changes = sum(diff(sign(error_diff)) ~= 0);
        
        if sign_changes == 0
            fprintf('  %s: строго монотонный\n', get_variable_name(i));
        elseif sign_changes < 3
            fprintf('  %s: слабо колебательный (%d перегиба)\n', get_variable_name(i), sign_changes);
        else
            fprintf('  %s: сильно колебательный (%d перегибов)\n', get_variable_name(i), sign_changes);
        end
    end
    
    % 4. Расчет различных метрик ошибок
    % Средняя абсолютная ошибка (MAE)
    mae = mean(abs(error), 1);
    % Среднеквадратичная ошибка (RMSE)
    rmse = sqrt(mean(error.^2, 1));
    % Максимальная ошибка
    max_error = max(abs(error), [], 1);
    
    fprintf('\nСредние метрики ошибок:\n');
    fprintf('Метрика\t\tx\t\ty\t\tz\t\tpsi\n');
    fprintf('MAE\t\t%.4f\t%.4f\t%.4f\t%.4f\n', mae(1), mae(2), mae(3), mae(4));
    fprintf('RMSE\t%.4f\t%.4f\t%.4f\t%.4f\n', rmse(1), rmse(2), rmse(3), rmse(4));
    fprintf('MaxErr\t%.4f\t%.4f\t%.4f\t%.4f\n', max_error(1), max_error(2), max_error(3), max_error(4));
    
    % Расчет квадратичного критерия J (интеграл заменен суммой)
    dt = diff(t);
    dt = [dt; dt(end)]; % Для последней точки используем тот же dt
    
    J_terms = zeros(size(X,1),1);
    for k = 1:size(X,1)
        J_terms(k) = error(k,:)*Q*error(k,:)' + U(k,:)*R*U(k,:)';
    end
    J = sum(J_terms .* dt);
    fprintf('\nКвадратичный критерий J = %.4f\n', J);
    
    % Построение графиков
    figure('Name', 'Метрики ошибок', 'Position', [100 100 1200 800]);
    
    % Графики ошибок по координатам
    subplot(2,2,1);
    plot(t, error(:,1), 'r', 'LineWidth', 1.5);
    hold on;
    plot(t, error(:,2), 'g', 'LineWidth', 1.5);
    plot(t, error(:,3), 'b', 'LineWidth', 1.5);
    
    % Добавляем линии 5% коридора
    if all(Xd(:,1) == Xd(1,1))
        yline(settling_threshold*abs(Xd(1,1)), 'r--');
        yline(-settling_threshold*abs(Xd(1,1)), 'r--');
    end
    if all(Xd(:,2) == Xd(1,2))
        yline(settling_threshold*abs(Xd(1,2)), 'g--');
        yline(-settling_threshold*abs(Xd(1,2)), 'g--');
    end
    if all(Xd(:,3) == Xd(1,3))
        yline(settling_threshold*abs(Xd(1,3)), 'b--');
        yline(-settling_threshold*abs(Xd(1,3)), 'b--');
    end
    
    title('Ошибки по координатам');
    xlabel('Время, с');
    ylabel('Ошибка');
    legend('x', 'y', 'z');
    grid on;
    
    subplot(2,2,2);
    plot(t, error(:,4), 'm', 'LineWidth', 1.5);
    if all(Xd(:,4) == Xd(1,4))
        yline(settling_threshold*abs(Xd(1,4)), 'm--');
        yline(-settling_threshold*abs(Xd(1,4)), 'm--');
    end
    title('Ошибка по углу \psi');
    xlabel('Время, с');
    ylabel('Ошибка \psi');
    grid on;
    
    % График квадратичного критерия J (накопленный)
    subplot(2,2,3);
    J_cumulative = cumsum(J_terms .* dt);
    plot(t, J_cumulative, 'k', 'LineWidth', 2);
    title('Накопленное значение критерия J');
    xlabel('Время, с');
    ylabel('J(t)');
    grid on;
    
    % График управления
    subplot(2,2,4);
    plot(t, U, 'LineWidth', 1.5);
    title('Сигналы управления');
    xlabel('Время, с');
    ylabel('U');
    legend('U1', 'U2', 'U3', 'U4');
    grid on;
end

function name = get_variable_name(idx)
    switch idx
        case 1
            name = 'x';
        case 2
            name = 'y';
        case 3
            name = 'z';
        case 4
            name = 'psi';
    end
end