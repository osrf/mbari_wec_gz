#!/usr/bin/python3
# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pandas as pd
import numpy as np


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def compute_n(P, V):
    x = np.log(V[:, np.newaxis])
    X = np.hstack((np.ones_like(x), x))
    y = np.log(P[:, np.newaxis])
    b = np.linalg.lstsq(X, y, rcond=None)[0]
    n = -b[1]
    return n


def compute_polytropic(csv, target_P):
    PV = pd.read_csv(csv).to_numpy()
    P = PV[:, 1]
    P *= 6894.757
    V = PV[:, 0]
    inc = np.where(np.diff(V) >= 0)
    dec = np.where(np.diff(V) < 0)
    n1 = compute_n(P[inc], V[inc])
    n2 = compute_n(P[dec], V[dec])
    P0, nearest_idx = find_nearest(P[inc], target_P)
    V0 = V[inc][nearest_idx]
    return float(n1), float(n2), float(P0), float(V0)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Compute polytropic curves for Pneumatic Spring')
    parser.add_argument('lower_pv_csv',
                        help='csv for PV curves in lower spring')
    parser.add_argument('upper_pv_csv',
                        help='csv for PV curves in upper spring')
    args, unknown = parser.parse_known_args()
    n1_l, n2_l, P0_l, V0_l = compute_polytropic(args.lower_pv_csv, target_P=1212000)
    print('Lower Polytropic Parameters:' +
          f'n1={n1_l:.04f} n2={n2_l:.04f} P0={P0_l:.00f} V0={V0_l:.04f}')
    n1_u, n2_u, P0_u, V0_u = compute_polytropic(args.upper_pv_csv, target_P=410000)
    print('Upper Polytropic Parameters:' +
          f'n1={n1_u:.04f} n2={n2_u:.04f} P0={P0_u:.00f} V0={V0_u:.04f}')


if __name__ == '__main__':
    main()
