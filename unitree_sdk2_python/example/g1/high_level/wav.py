import struct
import time

def read_wav(filename):
    try:
        with open(filename, 'rb') as f:
            def read(fmt):
                return struct.unpack(fmt, f.read(struct.calcsize(fmt)))

            # === Chunk Header ===
            chunk_id, = read('<I')
            if chunk_id != 0x46464952:  # "RIFF"
                print(f"[ERROR] chunk_id != 'RIFF': {hex(chunk_id)}")
                return [], -1, -1, False

            chunk_size, = read('<I')
            format_tag, = read('<I')
            if format_tag != 0x45564157:  # "WAVE"
                print(f"[ERROR] format != 'WAVE': {hex(format_tag)}")
                return [], -1, -1, False

            # === 查找fmt和data块 ===
            fmt_found = False
            data_found = False
            audio_format = None
            num_channels = None
            sample_rate = None
            byte_rate = None
            block_align = None
            bits_per_sample = None
            raw_pcm = None
            subchunk2_size = 0

            while not (fmt_found and data_found):
                try:
                    subchunk_id_data = f.read(4)
                    if not subchunk_id_data:
                        break
                    
                    subchunk_id = struct.unpack('<I', subchunk_id_data)[0]
                    subchunk_size_data = f.read(4)
                    if not subchunk_size_data:
                        break
                    
                    subchunk_size = struct.unpack('<I', subchunk_size_data)[0]
                    
                    if subchunk_id == 0x20746D66:  # "fmt "
                        # fmt块
                        fmt_found = True
                        audio_format, = read('<H')
                        num_channels, = read('<H')
                        sample_rate, = read('<I')
                        byte_rate, = read('<I')
                        block_align, = read('<H')
                        bits_per_sample, = read('<H')
                        
                        # 处理额外的fmt数据
                        if subchunk_size > 16:
                            f.seek(subchunk_size - 16, 1)
                            
                    elif subchunk_id == 0x61746164:  # "data"
                        # data块
                        data_found = True
                        subchunk2_size = subchunk_size
                        raw_pcm = f.read(subchunk2_size)
                        if len(raw_pcm) != subchunk2_size:
                            print("[ERROR] Failed to read full PCM data")
                            return [], -1, -1, False
                    else:
                        # 跳过其他块
                        f.seek(subchunk_size, 1)
                        # 如果块大小是奇数，WAV文件需要填充字节对齐
                        if subchunk_size % 2 == 1:
                            f.seek(1, 1)
                except Exception as e:
                    print(f"[ERROR] 解析WAV块时出错: {e}")
                    break

            if not fmt_found:
                print("[ERROR] 未找到fmt块")
                return [], -1, -1, False
                
            if not data_found:
                print("[ERROR] 未找到data块")
                return [], -1, -1, False

            if audio_format != 1:
                print(f"[ERROR] 不支持的音频格式: {audio_format} (需要PCM格式=1)")
                return [], -1, -1, False

            if bits_per_sample not in [8, 16, 24, 32]:
                print(f"[ERROR] 不支持的位深度: {bits_per_sample}")
                return [], -1, -1, False

            # 将PCM数据转换为16位整数列表
            pcm_list = []
            
            if bits_per_sample == 8:
                # 8位音频数据
                for i in range(subchunk2_size):
                    # 8位音频通常是无符号的
                    sample = struct.unpack('<B', raw_pcm[i:i+1])[0]
                    # 转换为有符号16位 (扩展到-32768到32767范围)
                    sample = (sample - 128) * 256
                    pcm_list.append(sample)
                    
            elif bits_per_sample == 16:
                # 16位音频数据 (标准格式)
                for i in range(0, len(raw_pcm), 2):
                    if i + 1 < len(raw_pcm):
                        sample = struct.unpack('<h', raw_pcm[i:i+2])[0]
                        pcm_list.append(sample)
                        
            elif bits_per_sample == 24:
                # 24位音频数据
                for i in range(0, len(raw_pcm), 3):
                    if i + 2 < len(raw_pcm):
                        # 24位数据是小端序的
                        sample = struct.unpack('<i', raw_pcm[i:i+3] + b'\x00')[0]
                        sample >>= 8  # 右移8位得到正确的24位值
                        # 转换为16位
                        sample = sample >> 8
                        pcm_list.append(sample)
                        
            elif bits_per_sample == 32:
                # 32位音频数据
                for i in range(0, len(raw_pcm), 4):
                    if i + 3 < len(raw_pcm):
                        # 32位浮点格式
                        sample = struct.unpack('<i', raw_pcm[i:i+4])[0]
                        # 转换为16位
                        sample = sample >> 16
                        pcm_list.append(sample)

            return pcm_list, sample_rate, num_channels, True

    except Exception as e:
        print(f"[ERROR] read_wave() failed: {e}")
        return [], -1, -1, False


def write_wave(filename, sample_rate, samples, num_channels=1):
    try:
        import array
        # 确保samples是int16类型的数组
        if not isinstance(samples, array.array) or samples.typecode != 'h':
            samples = array.array('h', samples)

        subchunk2_size = len(samples) * 2  # 每个样本2字节
        chunk_size = 36 + subchunk2_size

        with open(filename, 'wb') as f:
            # RIFF chunk
            f.write(struct.pack('<I', 0x46464952))  # "RIFF"
            f.write(struct.pack('<I', chunk_size))
            f.write(struct.pack('<I', 0x45564157))  # "WAVE"

            # fmt subchunk
            f.write(struct.pack('<I', 0x20746D66))  # "fmt "
            f.write(struct.pack('<I', 16))          # Subchunk1 size
            f.write(struct.pack('<H', 1))           # Audio format (PCM)
            f.write(struct.pack('<H', num_channels))
            f.write(struct.pack('<I', sample_rate))
            f.write(struct.pack('<I', sample_rate * num_channels * 2))  # byte_rate
            f.write(struct.pack('<H', num_channels * 2))                # block_align
            f.write(struct.pack('<H', 16))                              # bits per sample

            # data subchunk
            f.write(struct.pack('<I', 0x61746164))  # "data"
            f.write(struct.pack('<I', subchunk2_size))
            f.write(samples.tobytes())

        return True
    except Exception as e:
        print(f"[ERROR] write_wave() failed: {e}")
        return False


def play_pcm_stream(client, pcm_list, stream_name="example", chunk_size=24000, sleep_time=1.0, verbose=False, stop_event=None):
    """
    Play PCM audio stream (16-bit little-endian format), sending data in chunks.

    Parameters:
        client: An object with a PlayStream method
        pcm_list: list[int], PCM audio data in int16 format
        stream_name: Stream name, default is "example"
        chunk_size: Number of bytes to send per chunk, default is 24000 (0.75 seconds at 16kHz)
        sleep_time: Delay between chunks in seconds, default is 1.0
        stop_event: threading.Event object to signal stopping, default is None
    """
    # 将int16列表转换为字节数据
    import struct
    import array
    import time
    
    # 使用array模块高效地将int16列表打包为字节数据
    pcm_array = array.array('h', pcm_list)  # 'h' 表示有符号短整型 (int16)
    pcm_data = pcm_array.tobytes()
    
    stream_id = str(int(time.time() * 1000))  # Unique stream ID based on current timestamp
    offset = 0
    chunk_index = 0
    total_size = len(pcm_data)
    
    # 重试机制参数
    max_retries = 3
    retry_count = 0

    while offset < total_size:
        # 检查是否需要停止播放
        if stop_event and stop_event.is_set():
            print("[INFO] Audio playback stopped by external signal")
            break
            
        remaining = total_size - offset
        current_chunk_size = min(chunk_size, remaining)
        chunk = pcm_data[offset:offset + current_chunk_size]

        if verbose:
            # Print info about the current chunk
            print(f"[CHUNK {chunk_index}] offset = {offset}, size = {current_chunk_size} bytes")
            print("  First 10 samples (int16): ", end="")
            for i in range(0, min(20, len(chunk) - 1), 2):
                sample = struct.unpack_from('<h', chunk, i)[0]
                print(sample, end=" ")
            print()

        # Send the chunk with retry mechanism
        ret_code, _ = client.PlayStream(stream_name, stream_id, chunk)
        if ret_code != 0:
            retry_count += 1
            if retry_count <= max_retries:
                print(f"[WARNING] Failed to send chunk {chunk_index}, return code: {ret_code}, retry {retry_count}/{max_retries}")
                time.sleep(0.1 * retry_count)  # 逐渐增加等待时间
                continue  # 重试
            else:
                print(f"[ERROR] Failed to send chunk {chunk_index}, return code: {ret_code}")
                break  # 超过最大重试次数，退出
        else:
            print(f"[INFO] Chunk {chunk_index} sent successfully")
            retry_count = 0  # 成功发送后重置重试计数

        offset += current_chunk_size
        chunk_index += 1
        
        # 检查是否需要停止播放
        if stop_event and stop_event.is_set():
            print("[INFO] Audio playback stopped by external signal")
            break
            
        # 增加延迟以避免网络拥塞
        time.sleep(sleep_time)


def convert_wav(pcm_list, sample_rate, num_channels):
    """
    转换音频格式为16kHz单声道，使用高质量重采样算法
    
    Args:
        pcm_list: 原始PCM数据列表
        sample_rate: 原始采样率
        num_channels: 原始声道数
        
    Returns:
        转换后的PCM数据列表，如果失败则返回None
    """
    try:
        import numpy as np
        
        # 将PCM数据转换为numpy数组
        pcm_data = np.array(pcm_list, dtype=np.float32)  # 使用float32以提高处理精度
        print(f"[DEBUG] 原始音频数据长度: {len(pcm_data)} 个样本")
        
        # 处理多声道音频（提取左声道或混合所有声道）
        if num_channels > 1:
            print(f"[INFO] 将 {num_channels} 声道音频转换为单声道")
            # 重塑数组以便处理多声道
            pcm_data = pcm_data.reshape(-1, num_channels)
            # 混合所有声道为单声道（取平均值）
            pcm_data = np.mean(pcm_data, axis=1)
            print(f"[DEBUG] 转换为单声道后数据长度: {len(pcm_data)} 个样本")
        
        # 处理采样率转换（如果需要）
        if sample_rate != 16000:
            print(f"[INFO] 将 {sample_rate}Hz 音频转换为 16000Hz")
            # 使用更高质量的重采样算法
            original_length = len(pcm_data)
            converted_pcm_data = _resample_audio(pcm_data, sample_rate, 16000)
            pcm_data = converted_pcm_data
            new_length = len(pcm_data)
            print(f"[DEBUG] 重采样后数据长度: {new_length} 个样本 (原始: {original_length})")
        
        # 应用简单的增益以提高音频响度
        # 计算当前最大幅度
        max_amplitude = np.max(np.abs(pcm_data))
        if max_amplitude > 0:
            # 应用增益以更好地利用16位动态范围
            gain = 0.8 * 32767 / max_amplitude  # 保留20%余量防止削波
            pcm_data = pcm_data * gain
        
        # 转换回16位整数格式
        # 使用更平滑的量化方法以减少量化噪声
        pcm_data = np.clip(pcm_data, -32768, 32767)  # 限制在16位范围内
        pcm_data = pcm_data.astype(np.int16)
        
        # 转换回列表格式
        result = pcm_data.tolist()
        print(f"[DEBUG] 最终音频数据长度: {len(result)} 个样本")
        return result
        
    except Exception as e:
        print(f"[ERROR] 音频格式转换失败: {e}")
        import traceback
        traceback.print_exc()
        return None


def _resample_audio(audio_data, original_rate, target_rate):
    """
    使用更高质量的算法进行音频重采样
    
    Args:
        audio_data: 原始音频数据 (numpy array)
        original_rate: 原始采样率
        target_rate: 目标采样率
        
    Returns:
        重采样后的音频数据
    """
    try:
        import numpy as np
        from scipy import signal
        
        # 使用scipy的resample_poly进行高质量重采样
        # 这种方法比简单的插值提供更好的音频质量
        gcd = np.gcd(original_rate, target_rate)
        up = target_rate // gcd
        down = original_rate // gcd
        
        print(f"[INFO] 重采样比例: {up}:{down}")
        
        # 使用抗混叠滤波器进行重采样
        resampled_data = signal.resample_poly(audio_data, up, down)
        
        return resampled_data
        
    except ImportError:
        # 如果scipy不可用，使用之前的重采样方法
        print("[WARNING] scipy不可用，使用插值方法进行重采样")
        import numpy as np
        
        original_length = len(audio_data)
        target_length = int(original_length * target_rate / original_rate)
        
        # 如果目标长度与原始长度相同，则直接返回
        if target_length == original_length:
            return audio_data
        
        # 使用更高质量的重采样方法
        if target_length > original_length:
            # 上采样 - 使用三次样条插值
            from scipy.interpolate import interp1d
            original_indices = np.arange(original_length)
            target_indices = np.linspace(0, original_length - 1, target_length)
            
            # 使用三次样条插值进行上采样
            interpolator = interp1d(original_indices, audio_data, kind='cubic', fill_value='extrapolate')
            resampled_data = interpolator(target_indices)
        else:
            # 下采样 - 使用抗混叠滤波和高质量插值
            from scipy import signal
            # 计算抗混叠滤波器
            nyquist = min(original_rate, target_rate) / 2.0
            cutoff = nyquist * 0.9  # 保留90%的频率范围，获得更好的音质
            filter_order = 12  # 增加滤波器阶数以获得更好的滤波效果
            
            # 设计低通滤波器
            sos = signal.butter(filter_order, cutoff, btype='low', fs=original_rate, output='sos')
            filtered_data = signal.sosfilt(sos, audio_data)
            
            # 使用三次样条插值进行下采样（已经滤波过，避免混叠）
            from scipy.interpolate import interp1d
            original_indices = np.arange(original_length)
            target_indices = np.linspace(0, original_length - 1, target_length)
            
            interpolator = interp1d(original_indices, filtered_data, kind='cubic', fill_value='extrapolate')
            resampled_data = interpolator(target_indices)
        
        return resampled_data
    except Exception as e:
        print(f"[ERROR] 重采样过程中出错: {e}")
        # 出错时回退到简单的线性插值
        import numpy as np
        original_length = len(audio_data)
        target_length = int(original_length * target_rate / original_rate)
        
        indices = np.linspace(0, original_length - 1, target_length)
        resampled_data = np.interp(indices, np.arange(original_length), audio_data)
        return resampled_data


def get_wav_duration(filename):
    """
    获取WAV文件的时长（秒）
    
    Args:
        filename: WAV文件路径
        
    Returns:
        float: 音频时长（秒），如果出错则返回-1
    """
    try:
        with open(filename, 'rb') as f:
            def read(fmt):
                return struct.unpack(fmt, f.read(struct.calcsize(fmt)))

            # === Chunk Header ===
            chunk_id, = read('<I')
            if chunk_id != 0x46464952:  # "RIFF"
                return -1

            chunk_size, = read('<I')
            format_tag, = read('<I')
            if format_tag != 0x45564157:  # "WAVE"
                return -1

            # === 查找fmt和data块 ===
            fmt_found = False
            data_found = False
            sample_rate = None
            num_channels = None
            bits_per_sample = None
            subchunk2_size = 0

            while not (fmt_found and data_found):
                try:
                    subchunk_id_data = f.read(4)
                    if not subchunk_id_data:
                        break
                    
                    subchunk_id = struct.unpack('<I', subchunk_id_data)[0]
                    subchunk_size_data = f.read(4)
                    if not subchunk_size_data:
                        break
                    
                    subchunk_size = struct.unpack('<I', subchunk_size_data)[0]
                    
                    if subchunk_id == 0x20746D66:  # "fmt "
                        # fmt块
                        fmt_found = True
                        # 跳过前6个字节 (audio_format, num_channels)
                        f.seek(6, 1)
                        sample_rate, = read('<I')
                        # 跳过接下来的6个字节 (byte_rate, block_align)
                        f.seek(6, 1)
                        bits_per_sample, = read('<H')
                        
                        # 处理额外的fmt数据
                        if subchunk_size > 16:
                            f.seek(subchunk_size - 16, 1)
                            
                    elif subchunk_id == 0x61746164:  # "data"
                        # data块
                        data_found = True
                        subchunk2_size = subchunk_size
                        # 不需要读取实际数据，只需要大小
                        f.seek(subchunk_size, 1)
                    else:
                        # 跳过其他块
                        f.seek(subchunk_size, 1)
                        # 如果块大小是奇数，WAV文件需要填充字节对齐
                        if subchunk_size % 2 == 1:
                            f.seek(1, 1)
                except Exception as e:
                    break

            if not fmt_found or not data_found or sample_rate is None:
                return -1

            # 计算时长（秒）
            bytes_per_sample = bits_per_sample // 8
            if bytes_per_sample == 0 or num_channels == 0 or sample_rate == 0:
                return -1
                
            total_samples = subchunk2_size // (bytes_per_sample * num_channels)
            duration = total_samples / sample_rate
            return duration

    except Exception as e:
        return -1
