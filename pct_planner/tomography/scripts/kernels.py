import string
import numpy as np

try:
    import cupy as cp
    CUDA_AVAILABLE = cp.cuda.is_available()
except (ImportError, Exception):
    CUDA_AVAILABLE = False
    cp = np


def utils_point(resolution, n_row, n_col):
    util_preamble = string.Template(
        '''
        __device__ int getIndexLine(float16 x, float16 center)
        {
            int i = round((x - center) / ${resolution});
            return i;
        }

        __device__ int getIndexMap_1d(float16 x, float16 y, float16 cx, float16 cy)
        {
            // Return 1D index of a point (x, y) in a layer
            int idx_x = getIndexLine(x, cx) + ${n_row} / 2;
            int idx_y = getIndexLine(y, cy) + ${n_col} / 2;

            // Check if the index is inside the map
            if (idx_x < 0 || idx_x >= ${n_row} || idx_y < 0 || idx_y >= ${n_col})
            {
                return -1;
            }
            return ${n_col} * idx_x + idx_y;
        }

        __device__ int getIndexBlock_1d(int idx, int layer_n)
        {
            // Return 1D index of a point (x, y) in multi-layer map block
            return (int)${layer_size} * layer_n + idx;
        }

        __device__ static float atomicMaxFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fmaxf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }

        __device__ static float atomicMinFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fminf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }
        '''
    ).substitute(
        resolution=resolution,
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def utils_map(n_row, n_col):
    util_preamble=string.Template(
        '''
        __device__ int getIdxRelative(int idx, int dx, int dy) 
        {
            // Return 1D index of the relative point (x+dx, y+dy) in multi-layer map block
            int idx_2d = idx % (int)${layer_size};
            int idx_x = idx_2d / ${n_col};
            int idx_y = idx_2d % ${n_col};
            int idx_rx = idx_x + dx;
            int idx_ry = idx_y + dy;

            if ( idx_rx < 0 || idx_rx > (${n_row} - 1) ) 
                return -1;
            if ( idx_ry < 0 || idx_ry > (${n_col} - 1) )
                return -1;

            return ${n_col} * dx + dy + idx;
        }
        '''
    ).substitute(
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def tomographyKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    if CUDA_AVAILABLE:
        tomography_kernel = cp.ElementwiseKernel(
            in_params='raw U points, raw U center',
            out_params='raw U layers_g, raw U layers_c',
            preamble=utils_point(resolution, n_row, n_col),
            operation=string.Template(
                '''
                U px = points[i * 3];
                U py = points[i * 3 + 1];
                U pz = points[i * 3 + 2];

                int idx = getIndexMap_1d(px, py, center[0], center[1]);
                if ( idx < 0 )
                    return;
                for ( int s_idx = 0; s_idx < ${n_slice}; s_idx ++ )
                {
                    U slice = ${slice_h0} + s_idx * ${slice_dh};
                    if ( pz <= slice )
                        atomicMaxFloat(&layers_g[getIndexBlock_1d(idx, s_idx)], pz);
                    else
                        atomicMinFloat(&layers_c[getIndexBlock_1d(idx, s_idx)], pz);
                }
                '''
            ).substitute(
                n_slice=n_slice,
                slice_h0=slice_h0,
                slice_dh=slice_dh
            ),
            name='tomography_kernel'
        )
    else:
        # CPU fallback
        class TomographyCPUKernel:
            def __init__(self, resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
                self.resolution = resolution
                self.n_row = n_row
                self.n_col = n_col
                self.n_slice = n_slice
                self.slice_h0 = slice_h0
                self.slice_dh = slice_dh

            def __call__(self, points, center, layers_g, layers_c, size):
                points_flat = points.flatten()
                center_x = float(center[0])
                center_y = float(center[1])

                for i in range(size):
                    px = float(points_flat[i * 3])
                    py = float(points_flat[i * 3 + 1])
                    pz = float(points_flat[i * 3 + 2])

                    idx_x = int(np.round((px - center_x) / self.resolution)) + self.n_row // 2
                    idx_y = int(np.round((py - center_y) / self.resolution)) + self.n_col // 2

                    if idx_x < 0 or idx_x >= self.n_row or idx_y < 0 or idx_y >= self.n_col:
                        continue

                    idx = self.n_col * idx_x + idx_y

                    for s_idx in range(self.n_slice):
                        slice_h = self.slice_h0 + s_idx * self.slice_dh
                        block_idx = self.n_row * self.n_col * s_idx + idx

                        if pz <= slice_h:
                            layers_g.flat[block_idx] = max(layers_g.flat[block_idx], pz)
                        else:
                            layers_c.flat[block_idx] = min(layers_c.flat[block_idx], pz)

        tomography_kernel = TomographyCPUKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh)

    return tomography_kernel


def travKernel(
    n_row, n_col, half_kernel_size,
    interval_min, interval_free, step_cross, step_stand, standable_th, cost_barrier
    ):
    if CUDA_AVAILABLE:
        trav_kernel = cp.ElementwiseKernel(
            in_params='raw U interval, raw U grad_mag_sq, raw U grad_mag_max',
            out_params='raw U trav_cost',
            preamble=utils_map(n_row, n_col),
            operation=string.Template(
                '''
                if ( interval[i] < ${interval_min} )
                {
                    trav_cost[i] = ${cost_barrier};
                    return;
                }
                else
                    trav_cost[i] += max(0.0, 20 * (${interval_free} - interval[i]));
                if ( grad_mag_sq[i] <= ${step_stand_sq} )
                {
                    trav_cost[i] += 15 * grad_mag_sq[i] / ${step_stand_sq};
                    return;
                }
                else
                {
                    if ( grad_mag_max[i] <= ${step_cross_sq} )
                    {
                        int standable_grids = 0;
                        for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ )
                        {
                            for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ )
                            {
                                int idx = getIdxRelative(i, dx, dy);
                                if ( idx < 0 )
                                    continue;
                                if ( grad_mag_sq[idx] < ${step_stand_sq} )
                                    standable_grids += 1;
                            }
                        }
                        if ( standable_grids < ${standable_th} )
                        {
                            trav_cost[i] = ${cost_barrier};
                            return;
                        }
                        else
                            trav_cost[i] += 20 * grad_mag_max[i] / ${step_cross_sq};
                    }
                    else
                    {
                        trav_cost[i] = ${cost_barrier};
                        return;
                    }
                }
                '''
            ).substitute(
                half_kernel_size=half_kernel_size,
                interval_min=interval_min,
                interval_free=interval_free,
                step_cross_sq=step_cross ** 2,
                step_stand_sq=step_stand ** 2,
                standable_th=standable_th,
                cost_barrier=cost_barrier
            ),
            name='trav_kernel'
        )
    else:
        # CPU fallback
        class TravCPUKernel:
            def __init__(self, n_row, n_col, half_kernel_size, interval_min, interval_free,
                         step_cross, step_stand, standable_th, cost_barrier):
                self.n_row = n_row
                self.n_col = n_col
                self.half_kernel_size = half_kernel_size
                self.interval_min = interval_min
                self.interval_free = interval_free
                self.step_cross_sq = step_cross ** 2
                self.step_stand_sq = step_stand ** 2
                self.standable_th = standable_th
                self.cost_barrier = cost_barrier
                self.layer_size = n_row * n_col

            def __call__(self, interval, grad_mag_sq, grad_mag_max, trav_cost, size):
                for i in range(size):
                    if interval.flat[i] < self.interval_min:
                        trav_cost.flat[i] = self.cost_barrier
                        continue
                    else:
                        trav_cost.flat[i] += max(0.0, 20 * (self.interval_free - interval.flat[i]))

                    if grad_mag_sq.flat[i] <= self.step_stand_sq:
                        trav_cost.flat[i] += 15 * grad_mag_sq.flat[i] / self.step_stand_sq
                        continue
                    else:
                        if grad_mag_max.flat[i] <= self.step_cross_sq:
                            # Get 2D indices
                            idx_2d = i % self.layer_size
                            idx_x = idx_2d // self.n_col
                            idx_y = idx_2d % self.n_col

                            standable_grids = 0
                            for dy in range(-self.half_kernel_size, self.half_kernel_size + 1):
                                for dx in range(-self.half_kernel_size, self.half_kernel_size + 1):
                                    idx_rx = idx_x + dx
                                    idx_ry = idx_y + dy

                                    if idx_rx < 0 or idx_rx >= self.n_row or idx_ry < 0 or idx_ry >= self.n_col:
                                        continue

                                    relative_idx = self.n_col * dx + dy + i

                                    if grad_mag_sq.flat[relative_idx] < self.step_stand_sq:
                                        standable_grids += 1

                            if standable_grids < self.standable_th:
                                trav_cost.flat[i] = self.cost_barrier
                                continue
                            else:
                                trav_cost.flat[i] += 20 * grad_mag_max.flat[i] / self.step_cross_sq
                        else:
                            trav_cost.flat[i] = self.cost_barrier
                            continue

        trav_kernel = TravCPUKernel(n_row, n_col, half_kernel_size, interval_min, interval_free,
                                     step_cross, step_stand, standable_th, cost_barrier)

    return trav_kernel


def inflationKernel(n_row, n_col, half_kernel_size):
    if CUDA_AVAILABLE:
        inflation_kernel = cp.ElementwiseKernel(
            in_params='raw U trav_cost, raw U score_table',
            out_params='raw U inflated_cost',
            preamble=utils_map(n_row, n_col),
            operation=string.Template(
                '''
                int counter = 0;
                for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ )
                {
                    for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ )
                    {
                        int idx = getIdxRelative(i, dx, dy);
                        if ( idx >= 0 )
                            inflated_cost[i] = max(inflated_cost[i], trav_cost[idx] * score_table[counter]);
                        counter += 1;
                    }
                }
                '''
            ).substitute(
                half_kernel_size=half_kernel_size
            ),
            name='inflation_kernel'
        )
    else:
        # CPU fallback
        class InflationCPUKernel:
            def __init__(self, n_row, n_col, half_kernel_size):
                self.n_row = n_row
                self.n_col = n_col
                self.half_kernel_size = half_kernel_size
                self.layer_size = n_row * n_col

            def __call__(self, trav_cost, score_table, inflated_cost, size):
                for i in range(size):
                    # Get 2D indices
                    idx_2d = i % self.layer_size
                    idx_x = idx_2d // self.n_col
                    idx_y = idx_2d % self.n_col

                    counter = 0
                    for dy in range(-self.half_kernel_size, self.half_kernel_size + 1):
                        for dx in range(-self.half_kernel_size, self.half_kernel_size + 1):
                            idx_rx = idx_x + dx
                            idx_ry = idx_y + dy

                            if idx_rx >= 0 and idx_rx < self.n_row and idx_ry >= 0 and idx_ry < self.n_col:
                                relative_idx = self.n_col * dx + dy + i
                                inflated_cost.flat[i] = max(
                                    inflated_cost.flat[i],
                                    trav_cost.flat[relative_idx] * score_table.flat[counter]
                                )
                            counter += 1

        inflation_kernel = InflationCPUKernel(n_row, n_col, half_kernel_size)

    return inflation_kernel