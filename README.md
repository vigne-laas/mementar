# mementar

## Test the compression

You can test LZ77 or Huffman compression as follows:

./compress -i path/to/input_file -o path/to/output_file -lz
OR
./compress -i path/to/input_file -o path/to/output_file -hu

## Test the decompression

You can test LZ77 or Huffman uncompression as follows:

./uncompress -i path/to/input_file.mlz -o path/to/output_file -lz
OR
./uncompress -i path/to/input_file.mhu -o path/to/output_file -hu
