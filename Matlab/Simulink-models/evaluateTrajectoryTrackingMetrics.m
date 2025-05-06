function metrics = evaluateTrajectoryTrackingMetrics(X, Xd, U, t, Q, R)
    % Ошибка
    e = X - Xd;

    % Координаты (позиция)
    e_pos = e(:,1:3); % только r_x, r_y, r_z

    % Метрики
    mae = mean(abs(e_pos), 1);
    rmse = sqrt(mean(e_pos.^2, 1));
    mse = mean(e_pos.^2, 1);
    mean_err = mean(e_pos, 1);

    % Расчет квадратичного критерия J (интеграл заменен суммой)
    dt = diff(t);
    dt = [dt; dt(end)]; % Для последней точки используем тот же dt
    
    J_terms = zeros(size(X,1),1);
    for k = 1:size(X,1)
        J_terms(k) = e_pos(k,:)*Q*e_pos(k,:)' + U(k,:)*R*U(k,:)';
    end
    J = sum(J_terms .* dt);
    fprintf('\nКвадратичный критерий J = %.4f\n', J);


    % Показатель перерегулирования (overshoot)
    overshoot = zeros(1,3);
    for i = 1:3
        desired = Xd(:,i);
        actual = X(:,i);
        max_val = max(actual);
        overshoot(i) = (max_val - desired(end)) / abs(desired(end)) * 100;
    end

    % Время переходного процесса (5% коридор)
    % settling_time = zeros(1,3);
    % for i = 1:3
    %     ref = Xd(:,i);
    %     act = X(:,i);
    %     error_abs = abs(ref - act);
    %     threshold = 0.05 * abs(ref(end));
    %     idx = find(error_abs > threshold);
    %     if isempty(idx)
    %         settling_time(i) = 0;
    %     else
    %         last_out = idx(end);
    %         % проверим, остаётся ли ошибка в пределах 5% после последнего выхода
    %         sustained_within = all(error_abs(last_out+1:end) <= threshold);
    %         if sustained_within
    %             settling_idx = find(error_abs <= threshold & (1:length(t))' >= last_out, 1);
    %             disp(settling_idx)
    %             disp(t(settling_idx))
    %             settling_time(i) = t(settling_idx);
    %         else
    %             settling_time(i) = NaN; % не входит в допустимую зону
    %         end
    %     end
    % end

    % Сборка результатов
    metrics.MAE = mae;
    metrics.RMSE = rmse;
    metrics.MSE = mse;
    metrics.MeanError = mean_err;
    metrics.J = J;
    metrics.OvershootPercent = overshoot;
    % metrics.SettlingTimeSeconds = settling_time;

    % Пояснение
    fprintf('\n--- Оценка слежения за траекторией ---\n');
    fprintf('MAE (r_x, r_y, r_z): %.4f, %.4f, %.4f\n', mae);
    fprintf('RMSE (r_x, r_y, r_z): %.4f, %.4f, %.4f\n', rmse);
    fprintf('MSE (r_x, r_y, r_z): %.4f, %.4f, %.4f\n', mse);
    fprintf('Mean Error (r_x, r_y, r_z): %.4f, %.4f, %.4f\n', mean_err);
    
    fprintf('Перерегулирование (%%): %.2f, %.2f, %.2f\n', overshoot);
    % fprintf('Время переходного процесса (сек) [до 5%% и не выходит позже]:\n');
    % fprintf('  r_x: %.3f сек\n', settling_time(1));
    % fprintf('  r_y: %.3f сек\n', settling_time(2));
    % fprintf('  r_z: %.3f сек\n', settling_time(3));
end
