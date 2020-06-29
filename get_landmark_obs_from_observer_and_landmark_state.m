function [vector_to_centroid] = get_landmark_obs_from_observer_and_landmark_state(l, x)
    %l = [lx ly, lz]
    %x = [x, y, z, qx, qy, qz, qw]
    vector_to_centroid = l - x(1:3);
                          
    
    
end
