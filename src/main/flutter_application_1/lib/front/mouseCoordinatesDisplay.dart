import 'package:flutter/material.dart';

class mouseCoordinatesDisplay extends StatelessWidget {
  final double xMeters;
  final double yMeters;

  const mouseCoordinatesDisplay({
    super.key,
    required this.xMeters,
    required this.yMeters,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.9),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Text(
        _formatCoordinatesText(),
        style: const TextStyle(fontSize: 16),
      ),
    );
  }

  String _formatCoordinatesText() {
    return 'Mouse X: ${xMeters.toStringAsFixed(2)} m\n'
        'Mouse Y: ${yMeters.toStringAsFixed(2)} m';
  }
}