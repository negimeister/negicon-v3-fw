    trait SPIDevice {
    fn init() -> Result<(), ()>;
    fn poll() -> Result<(), ()>;
    fn detect(spi: SPIDevice<>)
}
