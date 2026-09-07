function wall = make_wall(center, normal)

center = center(:);
normal = normal(:);

if norm(normal) < 1e-12
    error('wall_normal must be nonzero');
end

wall.center = center;
wall.normal = normal / norm(normal);

end