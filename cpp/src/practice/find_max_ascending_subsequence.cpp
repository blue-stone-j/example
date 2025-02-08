/*
determine the longest contiguous subvector in A such that its elements form an ascending sequence and are not
"interrupted" by any element in B

Complexity
Time Complexity: O(∣A∣+∣B∣), since both A and B are traversed once.
Space Complexity: O(∣A∣), for storing the maxSubsequence.
*/

#include <iostream>
#include <set>
#include <vector>

std::vector<int> findMaxAscendingSubsequence(const std::set<int>& A, const std::set<int>& B) {
    std::vector<int> maxSubsequence; // Result
    std::vector<int> currentSubsequence; // Temporary to track current ascending subsequence

    auto itB = B.begin(); // Iterator for B
    for (auto itA = A.begin(); itA != A.end(); ++itA) {
        // Advance B's iterator to maintain sorted relationship
        while (itB != B.end() && *itB < *itA) {
            ++itB;
        }

        if (itB != B.end() && *itB == *itA) {
            // Update maxSubsequence if current is longer
            if (currentSubsequence.size() > maxSubsequence.size()) {
                maxSubsequence = currentSubsequence;
            }
            currentSubsequence.clear(); // Clear current subsequence as it's interrupted
        } else {
            if (currentSubsequence.empty() || *itA > currentSubsequence.back()) {
                currentSubsequence.push_back(*itA); // Add to current ascending subsequence
            } else {
                // Update maxSubsequence if current is longer
                if (currentSubsequence.size() > maxSubsequence.size()) {
                    maxSubsequence = currentSubsequence;
                }
                currentSubsequence = {*itA}; // Start a new subsequence
            }
        }
    }

    // Final check for the last subsequence
    if (currentSubsequence.size() > maxSubsequence.size()) {
        maxSubsequence = currentSubsequence;
    }

    return maxSubsequence;
}

int main() {
    std::set<int> A = {1, 2, 4, 5, 6, 7, 8, 9};
    std::set<int> B = {3, 6, 10};

    std::vector<int> result = findMaxAscendingSubsequence(A, B);

    std::cout << "Max ascending subsequence: ";
    for (int num : result) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    return 0;
}
