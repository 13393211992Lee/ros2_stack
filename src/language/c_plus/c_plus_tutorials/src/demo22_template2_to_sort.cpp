#include<iostream>
#include <string.h>
using namespace std;
// m  ,  <>
// 冒泡排序：函数模板
template<typename T>
void mySort(T arr[] ,size_t len){
  for (size_t i = 0; i < len-1; i++){
    for (size_t j = 0; j < len-i-1; j++){

      if (arr[j] > arr[j+1]) {
				T temp = arr[j + 1];
				arr[j + 1] = arr[j];
				arr[j] = temp;
      }
    }
  }
}


// 打印结果函数模板
template<typename T>
void printArr(T arr[] ,size_t len){
  for (size_t i = 0; i < len; i++)
	{
		cout << arr[i] << " ";
	}
	cout << endl;
}
int main()
{
  char tempChar[] = "abzab";
	size_t charLen = strlen(tempChar);

	
	mySort(tempChar, charLen);
	printArr(tempChar, charLen);

	int tempInt[] = {4,33,11,2,7};
	size_t intLen = sizeof(tempInt) / sizeof(tempInt[0]);

	mySort(tempInt, intLen);
	printArr(tempInt, intLen);

	return EXIT_SUCCESS;
}
