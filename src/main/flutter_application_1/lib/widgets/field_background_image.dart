class FieldBackgroundImage extends StatelessWidget {
  final double width;
  final double height;

  const FieldBackgroundImage({
    super.key,
    required this.width,
    required this.height,
  });

  @override
  Widget build(BuildContext context) {
    return Image.asset(
      FieldConstants.fieldImagePath,
      width: width,
      height: height,
      fit: BoxFit.cover,
    );
  }
}