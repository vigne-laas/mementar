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

## Test the archiving

Archive multiple files and a description file:

./archive -d path/to/description_file -i path/to/input_file_1 -i path/to/input_file_2 -o path/to/output_file

List the contents of an archive:

./archive -i path/to/archive.mar -l

Extract an archive into a folder:

./archive -i path/to/archive.mar -o out/path -x
