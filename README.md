# Notes
1. Notations and derivatives of the Error-State Kalman Filter are from the paper: http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
2. Majority of the codes in this repository are from the reference implementation (along with the test data) of https://github.com/je310/ESKF, but are modified for just focusing on the basic understanding of the ESKF theory.
3. Personal understanding of the ESKF theory is stated here (in Chinese): https://www.jianshu.com/p/531e91f87931. Just personal...
4. Appreciate any bug reports.


# Building 
1.  install the Eigen3
2.  mkdir build \
    cd build \
    cmake .. \
    make -j8

# Run
cd to ESKF_toy/bin \
./ESKF_toy ../dataset/gentleWave/ ../evaluation/result/
