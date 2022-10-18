use anyhow::Result;
use std::fs::File;
use std::io::BufWriter;
use png::{BitDepth, ColorType};
use std::path::Path;

pub struct Image<T> {
    /// Image data
    data: Vec<T>,
    /// Width * Stride
    row_size: usize,
    /// Number of colors per pixel
    stride: usize,
}

impl<T> Image<T> {
    pub fn new(data: Vec<T>, width: usize, stride: usize) -> Self {
        Self {
            data,
            row_size: width * stride,
            stride,
        }
    }

    pub fn height(&self) -> usize {
        self.data.len() / self.row_size
    }

    pub fn width(&self) -> usize {
        self.row_size / self.stride
    }

    pub fn data(&self) -> &[T] {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut [T] {
        &mut self.data
    }

    pub fn row_size(&self) -> usize {
        self.row_size
    }

    pub fn map<U, const N: usize>(&self, f: impl Fn(&[T]) -> [U; N]) -> Image<U> {
        let data = self
            .data()
            .chunks_exact(self.stride)
            .map(f)
            .flatten()
            .collect();
        Image::new(data, self.width(), N)
    }

    pub fn zip<U, V, const N: usize>(
        &self,
        other: &Image<V>,
        f: impl Fn(&[T], &[V]) -> [U; N],
    ) -> Image<U> {
        assert_eq!(self.width(), other.width());
        let data = self
            .data()
            .chunks_exact(self.stride)
            .zip(other.data().chunks_exact(other.stride))
            .map(|(a, b)| f(a, b))
            .flatten()
            .collect();
        Image::new(data, self.width(), N)
    }
}

pub fn load_color_png(path: impl AsRef<Path>) -> Result<Image<u8>> {
    let decoder = png::Decoder::new(File::open(path).unwrap());

    let mut reader = decoder.read_info().unwrap();

    let mut data = vec![0; reader.output_buffer_size()];

    let info = reader.next_frame(&mut data).unwrap();

    assert_eq!(info.bit_depth, BitDepth::Eight);
    assert_eq!(info.color_type, ColorType::Rgb);

    data.truncate(info.buffer_size());

    Ok(Image {
        data,
        stride: 3,
        row_size: info.width as usize * 3,
    })
}

pub fn load_depth_png(path: impl AsRef<Path>) -> Result<Image<u16>> {
    let decoder = png::Decoder::new(File::open(path).unwrap());

    let mut reader = decoder.read_info().unwrap();

    let mut data = vec![0; reader.output_buffer_size()];

    let info = reader.next_frame(&mut data).unwrap();

    assert_eq!(info.bit_depth, BitDepth::Sixteen);
    assert_eq!(info.color_type, ColorType::Grayscale);

    data.truncate(info.buffer_size());

    let data = bytemuck::cast_slice(&data).to_vec();

    Ok(Image {
        data,
        stride: 1,
        row_size: info.width as usize * 1,
    })
}

pub fn write_color_png(path: impl AsRef<Path>, image: &Image<u8>) -> Result<()> {
    let file = File::create(path)?;
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, image.width() as _, image.height() as _); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);

    let mut writer = encoder.write_header()?;

    writer.write_image_data(image.data())?;

    Ok(())
}
