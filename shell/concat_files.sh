# ls -v *.txt: Lists the .txt files in natural numeric order (1.txt, 2.txt, 10.txt, 20.txt, etc.).

# cat ... > combined.txt: Concatenates the contents into combined.txt.

cat $(ls -v *.txt) > combined.txt