function [points] = generate_shape(shape_type, radius, n_petals, num_points)
%GENERATE_SHAPE Genere les waypoints cartesiens pour les formes parametriques
%   Formes disponibles : 'square', 'fish', 'rose'
%
%   Entrees :
%       shape_type  - Chaine : 'square', 'fish', ou 'rose'
%       radius      - Rayon ou demi-dimension caracteristique (m)
%       n_petals    - Nombre de petales (utilise pour 'rose')
%       num_points  - Nombre total de waypoints a generer
%   Sortie :
%       points      - Matrice (num_points x 3) [X, Y, Z] centree a l'origine

    switch lower(shape_type)
        case 'square'
            pts_2d = generate_square(radius, num_points);
        case 'fish'
            pts_2d = generate_fish(radius, num_points);
        case 'rose'
            pts_2d = generate_rose(radius, n_petals, num_points);
        otherwise
            error('GENERATE_SHAPE:InvalidType', ...
                  'Type de forme "%s" inconnu. Choisir parmi : "square", "fish", "rose".', ...
                  shape_type);
    end

    % Coordonnees cartésiennes 3D [X, Y, Z] (forme dans le plan Z=0)
    points = [pts_2d, zeros(size(pts_2d, 1), 1)];
end

%% --- Fonctions locales de calcul geometrique ---

function pts = generate_square(radius, n)
    % Parcours ferme d'un carre de cote 2*radius dans le sens trigonométrique
    n_side = max(ceil(n / 4), 2);

    % Segment 1 : (-r, -r) -> (r, -r)
    s1 = [linspace(-radius, radius, n_side)', -radius * ones(n_side, 1)];
    % Segment 2 : (r, -r) -> (r, r)
    s2 = [radius * ones(n_side, 1), linspace(-radius, radius, n_side)'];
    % Segment 3 : (r, r) -> (-r, r)
    s3 = [linspace(radius, -radius, n_side)', radius * ones(n_side, 1)];
    % Segment 4 : (-r, r) -> (-r, -r)
    s4 = [-radius * ones(n_side, 1), linspace(radius, -radius, n_side)'];

    raw = [s1; s2(2:end,:); s3(2:end,:); s4(2:end,:)];

    % Re-echantillonnage uniforme au nombre exact demande
    if size(raw, 1) == n
        pts = raw;
    else
        t_old = linspace(0, 1, size(raw, 1))';
        t_new = linspace(0, 1, n)';
        pts = [interp1(t_old, raw(:,1), t_new), interp1(t_old, raw(:,2), t_new)];
    end
end

function pts = generate_fish(radius, n)
    % Courbe du poisson (Fish Curve parametrique fermee)
    % x(t) = cos(t) - sin^2(t)/sqrt(2)
    % y(t) = cos(t)*sin(t)
    t = linspace(0, 2*pi, n + 1)';
    t = t(1:end-1); % eviter la repetition du premier point

    x = cos(t) - (sin(t).^2) / sqrt(2);
    y = cos(t) .* sin(t);

    % Normalisation a l'echelle du rayon
    scale = max(sqrt(x.^2 + y.^2));
    if scale > 0
        x = (x / scale) * radius;
        y = (y / scale) * radius;
    end
    pts = [x, y];
end

function pts = generate_rose(radius, n_petals, n)
    % Rhodonee (Rose polaire) : r = radius * cos(k * theta)
    % Si k est impair : k petales. Si k est pair : 2k petales.
    theta = linspace(0, 2*pi, n + 1)';
    theta = theta(1:end-1);

    r = radius * cos(n_petals * theta);
    x = r .* cos(theta);
    y = r .* sin(theta);

    pts = [x, y];
end
