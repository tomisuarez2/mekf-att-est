function q_BE = TRIAD(t, acc, mag, m_E, g_E)
    % t   -> Nx1 timestamps
    % acc -> Nx3 accelerometer measurement (m/s^2)
    % mag -> Nx3 magnetometer measurement
    % m_E -> 1x3 inertial normalized magnetic field orientation
    % g_E -> 1x3 inertial normalized gravity orientation
    
    N = length(t);
    
    q_BE = zeros(N,4);
    
    % base inercial (TRIAD)
    t1_E = g_E;
    t2_E = cross(t1_E,m_E);
    t2_E = t2_E / norm(t2_E);
    t3_E = cross(t1_E,t2_E);
    
    T_E = [t1_E t2_E t3_E];
    
    for k = 1:N
        
        a_B = acc(k,:)';
        m_B = mag(k,:)';
        
        % normalizacion
        a_B = a_B / norm(a_B);
        m_B = m_B / norm(m_B);
        
        % TRIAD en cuerpo
        t1_B = -a_B;
        
        t2_B = cross(t1_B,m_B);
        t2_B = t2_B / norm(t2_B);
        
        t3_B = cross(t1_B,t2_B);
        
        T_B = [t1_B t2_B t3_B];
        
        % matriz de rotacion
        R_BE = T_B * T_E';
        
        % conversion a cuaternion
        q = dcm_to_quat(R_BE);
        
        % normalizar
        q = q / norm(q);
        
        q_BE(k,:) = q;
        
    end

end

