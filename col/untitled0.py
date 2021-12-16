# -*- coding: utf-8 -*-
"""
Created on Thu Dec 16 01:41:13 2021

@author: zzhou
"""

import os, glob
import pandas as pd

all_files = glob.glob("test*.csv")
df_from_each_file = (pd.read_csv(f, sep=',') for f in all_files)
df_merged   = pd.concat(df_from_each_file, ignore_index=True)
df_merged.to_csv( "merged.csv")