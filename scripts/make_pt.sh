echo "Generating OLAP page table traces"
python3 examples/make_pt.py --kernel imdb_lt_int64 --input_dir ./outputs/imdb_lt_int64/outputs --output_dir ./traces/imdb_lt_int64/0
python3 examples/make_pt.py --kernel imdb_gteq_lt_int64 --input_dir ./outputs/imdb_gteq_lt_int64/outputs --output_dir ./traces/imdb_gteq_lt_int64/0 
python3 examples/make_pt.py --kernel imdb_gt_lt_fp32 --input_dir ./outputs/imdb_gt_lt_fp32/outputs --output_dir ./traces/imdb_gt_lt_fp32/0
python3 examples/make_pt.py --kernel imdb_three_col_and --input_dir ./outputs/imdb_three_col_and/outputs --output_dir ./traces/imdb_three_col_and/0