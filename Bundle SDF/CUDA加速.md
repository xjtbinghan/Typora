## 函数说明	

### cudaMalloc, cudaMemset,  cudaMemcpy（运行时函数）

​		cudaMalloc, cudaMemset, 和 cudaMemcpy 都是 NVIDIA CUDA 编程中使用的函数，用于在GPU上管理内存。这些函数不是CUDA核函数，而是在CUDA C/C++代码中由主机CPU调用的CUDA运行时函数。这些函数用于分配和释放设备内存，初始化设备内存的值以及在主机和设备之间传输数据。它们的主要区别如下：

`cudaMalloc`:

- 功能：==用于在GPU上分配设备内存==。
- 参数：需要传递一个指向指针的指针，以及要分配的内存大小。
- 示例：

```c++
float* deviceArray;
cudaMalloc((void**)&deviceArray, size);
```

`cudaMemset`:

- 功能：==用于将设备内存的值设置为指定的常量==。

- 参数：需要传递一个指向设备内存的指针、要设置的值，以及要设置的内存大小。

- 示例：

    ```c++
    cudaMemset(deviceArray, 0, size);
    ```

`cudaMemcpy`:

- 功能：==用于在主机（CPU）和设备（GPU）之间复制数据==。

- 参数：需要传递目标指针（主机或设备）、源指针（主机或设备）、要复制的数据大小，以及复制的方向（从主机到设备、从设备到主机等）。

- 示例：

    ```c++
    cudaMemcpy(deviceArray, hostArray, size, cudaMemcpyHostToDevice);
    ```

#### 总结

- 使用 `cudaMalloc` 在GPU上分配一块内存，通常用于存储设备数据。
- 使用 `cudaMemset` 可以将设备内存中的数据初始化为特定的值，如零。
- 使用 `cudaMemcpy` 可以在主机和设备之间复制数据，使主机和设备之间能够共享数据。



### cudaStreamCreate（运行时函数）

​		在CUDA编程中，CUDA流（CUDA streams）是一种用于并行执行GPU任务的机制。CUDA流允许多个GPU操作在不同的流中同时执行，以充分利用GPU并行性，从而提高性能。每个CUDA流都代表了一系列GPU操作，这些操作可以按照特定的顺序或并行执行。==在代码中，作者对m个帧对进行RANSAC操作时，让每一个帧对都创建了自己的CUDA流，用来并行。（虽然代码中m=1）==

`cudaStreamCreate` 是一个CUDA运行时函数，用于创建CUDA流。它的原型如下：

```c++
cudaError_t cudaStreamCreate(cudaStream_t *stream);
```

- `stream` 是一个`cudaStream_t`类型的指针，用于存储新创建的CUDA流的句柄。

- `cudaStream_t` 是CUDA流的数据类型，表示CUDA流的句柄。您可以使用这个句柄来将操作放入特定的流中，以实现并行执行。

    ​		**通过使用不同的CUDA流，您可以在GPU上并行执行多个操作，而无需等待前一个操作完成。这对于在GPU上同时执行多个任务非常有用，尤其是在需要最大程度利用GPU并行性的情况下。**



### 核函数	

​		CUDA核函数是在GPU上并行执行的函数，它们使用特殊的语法 `<<<...>>>` 来指示GPU如何执行它们的工作。这些核函数由主机CPU启动并在GPU上运行。

*CUDA 流* ： cudaStream_t  ， cudaStreamCreate     

CUDA核函数：ransacEvalModelKernel